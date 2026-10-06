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

typedef enum {
    DCI_ERROR_WRITE_COMMAND = 19,
    DCI_ERROR_WRITE_TIMEOUT = 20,
    DCI_ERROR_READ_TIMEOUT = 21,
    DCI_RESPONSE_OK        = 22,
} dci_error;

const uint SWD_RETRY_COUNT = 200;
const uint DCI_RETRY_COUNT = 1001000;

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

/// DCI register to write command
#define DCI_WDATA       (0x00001000)

/// DCI register to read response
#define DCI_RDATA       (0x00001004)

/// DCI register to read status
#define DCI_STATUS      (0x00001008)

/// Response from the DCI is valid
#define RDATAVALID      (0x0100)

/// Write Request to the DCI is pending
#define WPENDING        (0x01)

#define ERROR_CODE_MASK         (0xff00)
#define ERROR_CODE_SHIFT        (8)

#define SECURE_DEBUG_MASK       (0x04)
#define SECURE_BOOT_MASK        (0x01)
#define SECURE_BOOT_NOT_CONF    (0xffffffff)
#define BOOT_STATUS_MASK        (0xff)
#define BOOT_MAIN_LOOP          (0x20)
#define DEVICE_ERASE_MASK       (0x02)
#define DEBUG_LOCK_MASK         (0x01)
#define DEBUG_LOCK_STATE_MASK   (0x20)
#define SE_BOOT_ERR_VER         (0x00010020)
#define SE_VER_MASK             (0x00ffffff)
#define NO_MCU_VER              (0xffffffff)

/// AHB-AP registers
#define AP_CSW          0
#define AP_TAR          1
#define AP_DRW          3
#define AP_IDR          3       // In bank 0xf

// response with tamper bits information
#define RESP_WITH_TAMPER  40

uint32_t cmd_buf[2];   // no argument cmd buffer
uint32_t cmd_resp[30];

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
    uint32_t swd_status = SWD_ERROR_OK;
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
  uint32_t swd_status = SWD_ERROR_OK;
  uint32_t retry = SWD_RETRY_COUNT;

  do {
    swd_status = write_reg(false, reg, data, false);
    retry--;
  } while ((swd_status == SWD_ERROR_WAIT) && (retry > 0));


  return (swd_status);
}

swd_error read_ap(int reg, uint32_t *data)
{
    uint32_t swd_status = SWD_ERROR_OK;
    uint32_t retry = SWD_RETRY_COUNT;

    do {
	swd_status = read_reg(true, reg, data);
	retry--;
    } while ((swd_status == SWD_ERROR_WAIT) && (retry > 0));


    return (swd_status);
}


swd_error write_ap(int reg, uint32_t data)
{
  uint32_t swd_status = SWD_ERROR_OK;
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

dci_error write_dci_command(uint32_t *command) {
    // first "word" (4 bytes) is length in bytes including that of the
    // first word.
    uint32_t value = 0;
    uint32_t retry = DCI_RETRY_COUNT;
    size_t count = command[0] / 4;

    // write command into dci register
    while (count--) {
	do {
	    write_ap(1, DCI_STATUS);
	    read_ap(3, &value);
	    read_dp(3, &value);
	    if ((value & RDATAVALID) != 0) {
		return (DCI_ERROR_WRITE_COMMAND);
	    }
	    retry--;
	} while (((value & WPENDING) != 0) && (retry > 0));

	if (retry == 0) {
	    return (DCI_ERROR_WRITE_TIMEOUT);
	}

	// Write 32-bit command word
	write_ap(1, DCI_WDATA);
	write_ap(3, *command++);
    }
}

dci_error read_dci_response(uint32_t *resp)
{
    uint32_t count;
    uint32_t retry = DCI_RETRY_COUNT;

    // Poll status to wait RDATAVALID to high
    do {
	write_ap(1, DCI_STATUS);
	read_ap(3, resp);
	read_dp(3, resp);
	retry--;
    } while (((*resp & RDATAVALID) != RDATAVALID) && (retry > 0));

    if (retry == 0) {
	return (DCI_ERROR_READ_TIMEOUT);
    }

    // Read the first response word from DCI register
    write_ap(1, DCI_RDATA);
    read_ap(3, resp);
    read_dp(3, resp);

    // Check response code, raise error if not 0
    if ((*resp >> 16) != 0) {
	return ((*resp >> 16) + DCI_RESPONSE_OK);
    }

    // Get the total length of the response word (total in bytes/4)
    count = (*resp & 0x00ff) >> 2;

    // Read the sequential response words
    while (--count != 0) {
	// Poll status to wait RDATAVALID to high
	resp++;
	do {
	    write_ap(1, DCI_STATUS);
	    read_ap(3, resp);
	    read_dp(3, resp);
	    retry--;
	} while (((*resp & RDATAVALID) != RDATAVALID) && (retry > 0));

	if (retry == 0) {
	    return (DCI_ERROR_READ_TIMEOUT);
	}

	// Read 32-bit response word
	write_ap(1, DCI_RDATA);
	read_ap(3, resp);
	read_dp(3, resp);
    }
}

char *get_error_string(uint32_t code)
{
    switch (code) {
    case SWD_ERROR_OK:
	return "No error.";
    case SWD_ERROR_WAIT:
	return "Timed out while waiting for WAIT response.";
    case SWD_ERROR_FAULT:
	return "Target returned FAULT response.";
    case SWD_ERROR_PROTOCOL:
	return "Protocol error, target does not respond.";
    case SWD_ERROR_PARITY:
	return "Parity error.";
    default:
	return "unknown error";
  }
}

static void print_se_status(uint32_t *cmd_resp_buf)
{
    uint32_t index;
    uint32_t boot_status;
    bool boot_error = false;

    // Check response from HSE or VSE
    if (cmd_resp_buf[0] == RESP_WITH_TAMPER) {
	index = 5;
    } else {
	index = 1;
    }

    // Save boot status
    boot_status = cmd_resp_buf[index++];

    // No boot status error code on SE firmware V2
    if ((cmd_resp_buf[index] & ~SE_VER_MASK) < 0x02000000) {
	// Boot status error code is available if SE firmware >= v1.2.0
	if ((cmd_resp_buf[index] & SE_VER_MASK) >= SE_BOOT_ERR_VER) {
	    boot_error = true;
	}
    }

    cmd_resp_buf[index] &= SE_VER_MASK;
    printf("OK\n");
    printf("  + SE firmware version  : %08lX\n", cmd_resp_buf[index++]);

    if (cmd_resp_buf[index] != NO_MCU_VER) {
	printf("  + MCU firmware version : %08lX\n", cmd_resp_buf[index]);
    } else {
	printf("  + MCU firmware version : NA\n");
    }

    index++;
    printf("  + Debug lock           : ");
    if (cmd_resp_buf[index] & DEBUG_LOCK_MASK) {
	printf("Enabled\n");
    } else {
	printf("Disabled\n");
    }

    printf("  + Debug lock state     : ");
    if (cmd_resp_buf[index] & DEBUG_LOCK_STATE_MASK) {
	printf("True\n");
    } else {
	printf("False\n");
    }

    printf("  + Device Erase         : ");
    if (cmd_resp_buf[index] & DEVICE_ERASE_MASK) {
	printf("Enabled\n");
    } else {
	printf("Disabled\n");
    }

    printf("  + Secure debug         : ");
    if (cmd_resp_buf[index] & SECURE_DEBUG_MASK) {
	printf("Enabled\n");
    } else {
	printf("Disabled\n");
    }

    index++;
    printf("  + Secure boot          : ");
    if (cmd_resp_buf[index] == SECURE_BOOT_NOT_CONF) {
	printf("Disabled and SE OTP is not configured\n");
    } else {
	if (cmd_resp_buf[index] & SECURE_BOOT_MASK) {
	    printf("Enabled\n");
	} else {
	    printf("Disabled\n");
	}
    }

    // Get boot status error code
    if (boot_error) {
	index = boot_status;
	boot_status = ((boot_status & ERROR_CODE_MASK) >> ERROR_CODE_SHIFT) + DCI_RESPONSE_OK;
	printf("  + Boot status          : %#lx - %s\n",
	       index & BOOT_STATUS_MASK, get_error_string(boot_status));
    } else {
	if ((boot_status & BOOT_STATUS_MASK) != BOOT_MAIN_LOOP) {
	    printf("  + Boot status          : %#lx - Failed\n",
		   boot_status & BOOT_STATUS_MASK);
	} else {
	    printf("  + Boot status          : %#lx - OK\n",
		   boot_status & BOOT_STATUS_MASK);
	}
    }
}

void get_status() {
    // 1. write the GET STATUS command
    cmd_buf[0] = 0x00000008UL;
    cmd_buf[1] = 0xFE010000UL;
    write_dci_command(&cmd_buf[0]);

    // 2. read the response and print it.
    read_dci_response(&cmd_resp[0]);

    print_se_status(&cmd_resp[0]);
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
    // read section "4.2 DCI Registers", 4.3 in AN1303 and "Get
    // Status" in section 6.9.
    get_status();
}
