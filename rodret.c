#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

const uint LED_PIN = 25;

const uint SWDIO_PIN  = 2; // GP2 - physical pin 4.
const uint SWDCLK_PIN = 3; // GP3 - physical pin 5.
const uint RESET_PIN  = 4; // GP4 - physical pin 6.
// note: on target it is nRESET (i.e. active low will reset the chip)

const uint ACK_OK    = 1;
const uint ACK_WAIT  = 2;
const uint ACK_FAULT = 4;

typedef enum {
    SWD_ERROR_OK = 1,
    SWD_ERROR_WAIT,
    SWD_ERROR_FAULT,
    SWD_ERROR_PROTOCOL,
    SWD_ERROR_PARITY,
} swd_error;

const uint SWD_RETRY_COUNT = 200;

/// DP addresses
/// Address of DP read registers
#define DP_IDCODE       0
#define DP_CTRL         1
#define DP_RESEND       2
#define DP_RDBUFF       3

/// Addresses of DP write registers
#define DP_ABORT        0
#define DP_STAT         1
#define DP_SELECT       2

/// Bit fields for the ABORT register
#define DP_ABORT_ORUNERRCLR     (1U << 4)
#define DP_ABORT_WDERRCLR       (1U << 3)
#define DP_ABORT_STKERRCLR      (1U << 2)
#define DP_ABORT_STKCMPCLR      (1U << 1)

/// Power up request and acknowledge bits in CTRL/STAT
#define DP_CTRL_CDBGPWRUPREQ    (1U << 28)
#define DP_CTRL_CDBGPWRUPACK    (1U << 29)
#define DP_CTRL_CSYSPWRUPREQ    (1U << 30)
#define DP_CTRL_CSYSPWRUPACK    (1U << 31)

/// Bit fields for the CSW register
#define AP_CSW_32BIT_TRANSFER   (0x02)
#define AP_CSW_AUTO_INCREMENT   (0x10)
#define AP_CSW_MASTERTYPE_DEBUG (1 << 29)
#define AP_CSW_HPROT            (1 << 25)
#define AP_CSW_DEFAULT          (AP_CSW_32BIT_TRANSFER | AP_CSW_MASTERTYPE_DEBUG | AP_CSW_HPROT)

/// DCI AP register bank 0
#define DCI_AP_REG      (0x01000000)

/// AHB-AP registers
#define AP_CSW          0
#define AP_TAR          1
#define AP_DRW          3
#define AP_IDR          3       // In bank 0xf

void write_bit_swd(int bit) {
    if (bit > 0) {
	gpio_put(SWDIO_PIN, 1);
    } else {
	gpio_put(SWDIO_PIN, 0);
    }
}

uint32_t read_bit_swd(void) {
    return gpio_get(SWDIO_PIN);
}

void swdio_set_output(void) {
    gpio_set_dir(SWDIO_PIN, GPIO_OUT);
}

void swdio_set_input(void) {
    gpio_set_dir(SWDIO_PIN, GPIO_IN);
}

void swdclk_cycle(void) {
    gpio_put(SWDCLK_PIN, 0);
    // delay??
    gpio_put(SWDCLK_PIN, 1);
}

void jtag_to_swd_seq(void) {
    swdio_set_output();

    // reset the line with > 50 cycles with swdio high
    write_bit_swd(1);
    for (size_t i = 0; i < 60; i++) {
	swdclk_cycle();
    }

    // tx 16-bit JTAG-TO-SWD sequence (lsb first)
    uint16_t val = 0xE79E;
    for (size_t i = 0; i < 16; i++) {
	uint8_t bit = (val >> i) & 1;
	write_bit_swd(bit);
    }

    // do another reset
    write_bit_swd(1);
    for (size_t i = 0; i < 60; i++) {
	swdclk_cycle();
    }

    // insert 16-cycle idle period.
    write_bit_swd(0);
    for (size_t i = 0; i < 16; i++) {
	swdclk_cycle();
    }
}

static swd_error read_reg(bool ap, int reg, uint32_t *data)
{
    uint32_t cb = 0;
    uint32_t ack = 0;
    uint32_t ret = SWD_ERROR_OK;
    uint32_t parity = 0;

    // Initialize output variable
    *data = 0;

    // Convert to integer
    int int_ap = (int)ap;
    int int_read = (int)1;

    int addr2 = reg & 0x01;
    int addr3 = (reg >> 1) & 0x01;

    // Calculate parity
    parity = (int_ap + int_read + addr2 + addr3) & 0x01;

    swdio_set_output();

    // Send request
    write_bit_swd(1);
    write_bit_swd(int_ap);
    write_bit_swd(int_read);
    write_bit_swd(addr2);
    write_bit_swd(addr3);
    write_bit_swd(parity);
    write_bit_swd(0);
    write_bit_swd(1);

    // Turn-around
    swdio_set_input();
    swdclk_cycle();

    // Read ACK
    for(size_t i = 0; i < 3; i++) {
	uint32_t b = read_bit_swd();
	ack |= b << i;
    }

    // Verify that ACK is OK
    if (ack == ACK_OK) {
	for(size_t i = 0; i < 32; i++) {
	    // Read bit
	    uint32_t b = read_bit_swd();
	    *data |= b << i;

	    // Keep track of expected parity
	    if (b) {
		cb = !cb;
	    }
	}

	// Read parity bit
	uint32_t parity = read_bit_swd();

	// Verify parity
	if (cb == parity) {
	    ret = SWD_ERROR_OK;
	} else {
	    ret = SWD_ERROR_PARITY;
	}
    } else if (ack == ACK_WAIT) {
	ret = SWD_ERROR_WAIT;
    } else if (ack == ACK_FAULT) {
	ret = SWD_ERROR_FAULT;
    } else {
	// Line not driven, protocol error
	ret = SWD_ERROR_PROTOCOL;
    }

    // Turn-around
    swdclk_cycle();

    // The 8-cycle idle period to make sure transaction is clocked through DAP
    swdio_set_output();
    for (size_t i = 0; i < 8; i++) {
	write_bit_swd(0);
    }

    return ret;
}

static swd_error write_reg(bool ap, int reg, uint32_t data, bool ignore_ack)
{
    uint32_t ack = 0;
    uint32_t parity = 0;
    uint32_t ret = SWD_ERROR_OK;

    // Convert to integer
    int int_ap = (int)ap;
    int int_read = (int)0;

    // Calculate address bits
    int addr2 = reg & 0x01;
    int addr3 = (reg >> 1) & 0x01;

    // Calculate parity
    parity = (int_ap + int_read + addr2 + addr3) & 0x01;

    swdio_set_output();

    // Write request
    write_bit_swd(1);
    write_bit_swd(int_ap);
    write_bit_swd(int_read);
    write_bit_swd(addr2);
    write_bit_swd(addr3);
    write_bit_swd(parity);
    write_bit_swd(0);
    write_bit_swd(1);

    swdio_set_input();

    // Turn-around
    swdclk_cycle();

    // Read acknowledge
    for (size_t i = 0; i < 3; i++) {
	uint32_t b = read_bit_swd();
	ack |= b << i;
    }

    if (ack == ACK_OK || ignore_ack) {
	// Turn-around
	swdclk_cycle();

	swdio_set_output();

	// Write data
	parity = 0;
	for (size_t i = 0; i < 32; i++) {
	    int b = (data >> i) & 0x01;
	    write_bit_swd(b);
	    if (b) {
		parity = !parity;
	    }
	}

	// Write parity bit
	write_bit_swd(parity);
    } else if (ack == ACK_WAIT) {
	ret = SWD_ERROR_WAIT;
    } else if (ack == ACK_FAULT) {
	ret = SWD_ERROR_FAULT;
    } else {
	// Line not driven, protocol error
	ret = SWD_ERROR_PROTOCOL;
    }

    // The 8-cycle idle period to make sure transaction is clocked through DAP
    swdio_set_output();
    for (size_t i = 0; i < 8; i++) {
	write_bit_swd(0);
    }

    return ret;
}

swd_error read_dp(int reg, uint32_t *data) {
    uint32_t swd_status;
    uint32_t retry = SWD_RETRY_COUNT;

    do {
	swd_status = read_reg(false, reg, data);
	retry--;
    } while (swd_status == SWD_ERROR_WAIT && retry > 0);

    if (swd_status != SWD_ERROR_OK) {
	return (swd_status);
    }
}

swd_error write_dp(int reg, uint32_t data)
{
  uint32_t swd_status;
  uint32_t retry = SWD_RETRY_COUNT;

  do {
    swd_status = write_reg(false, reg, data, false);
    retry--;
  } while ((swd_status == SWD_ERROR_WAIT) && (retry > 0));


  return (swd_status);
}

swd_error write_ap(int reg, uint32_t data)
{
  uint32_t swd_status;
  uint32_t retry = SWD_RETRY_COUNT;

  do {
    swd_status = write_reg(true, reg, data, false);
    retry--;
  } while ((swd_status == SWD_ERROR_WAIT) && (retry > 0));

  return (swd_status);
}

int connect_to_dci(void) {
    uint32_t id_code = 0;

    // send jtag to swd sequence via swdio pin.
    jtag_to_swd_seq();

    // read IDCODE (idcode reg = 0)
    read_dp(0, &id_code);

    // verify if id_code is 0x6BA02477
    if (id_code != 0x6BA02477) {
	// print something on uart and return
	puts("wrong IDCODE!");
	return (-1);
    }

    // clear (ABORT/DP reg 0) to clear error/sticky flags
    write_dp(DP_ABORT, DP_ABORT_ORUNERRCLR | DP_ABORT_WDERRCLR
	     | DP_ABORT_STKERRCLR | DP_ABORT_STKCMPCLR);

    // (STAT/DP reg 4) power up system and debug
    write_dp(DP_CTRL, DP_CTRL_CSYSPWRUPREQ | DP_CTRL_CDBGPWRUPREQ);

    // select (DP reg 0) to route interface to DCI (AP 1)
    write_dp(DP_SELECT, DCI_AP_REG);

    // Set transfer size to 32 bit
    write_ap(AP_CSW, AP_CSW_DEFAULT);
}

int disable_secure_debug(void) {
    // write 0x0000_0008, 0x430E_0000 (i.e. overall length (8 bytes,
    // including the first word which is length of 4 bytes) followed
    // by command without payload, which is another 4 bytes.

    // after sending this, we should read the status and make sure
    // there is no error.
}

// XXX: imlement erase_device command, do hard reset.

int main(void) {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_init(SWDIO_PIN);
    gpio_init(SWDCLK_PIN);
    gpio_init(RESET_PIN);

    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_set_dir(SWDCLK_PIN, GPIO_OUT);

    // connect to target and print SE status
    connect_to_dci();

    // prepare command buffer for GET_SE_STATUS command. It should
    // show that the secure debug is disabled.
    //
    // XXX
}
