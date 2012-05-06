#include <stdio.h>
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>

#include <libusb.h>

#define VERSION "0.02"

#define VENDOR_ID_MICROCHIP         0x04D8
#define PRODUCT_ID_USB_FIFO         0x0400

#define EP_CMD_BULK_IN              (1 | LIBUSB_ENDPOINT_IN)
#define EP_CMD_BULK_OUT             (1 | LIBUSB_ENDPOINT_OUT)

#define EP_DATA_BULK_IN             (2 | LIBUSB_ENDPOINT_IN)
#define EP_DATA_BULK_OUT            (2 | LIBUSB_ENDPOINT_OUT)

#define BUF_LEN                     64

static struct libusb_device_handle *devh = NULL;

static struct libusb_transfer *cmd_in_transfer = NULL;
static unsigned char cmd_in_buf[BUF_LEN];

static struct libusb_transfer *data_in_transfer = NULL;
static unsigned char data_in_buf[BUF_LEN];

static void print_buffer(unsigned char *buf, int cnt)
{
    int j;

    printf("Received data: %d bytes\n", cnt);
    printf("HEX:\n");
    for (j=0;j<cnt;++j) {
        if (j%16 == 0 && j != 0)
            printf("\n");
        printf("%02X ", (unsigned int)buf[j]);
    }
    printf("\nCHAR:\n");
    for (j=0;j<cnt;++j) {
        if (j%16 == 0 && j != 0)
            printf("\n");
        printf("%c", (unsigned int)buf[j]);
    }
    printf("\n");
}

static void cmd_in_cb(struct libusb_transfer *transfer)
{
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "CMD IN transfer status %d\n", transfer->status);
        exit(1);
	}

	printf("CMD IN transfer callback\n");
    print_buffer(transfer->buffer, transfer->actual_length);

	if (libusb_submit_transfer(cmd_in_transfer) < 0) {
        printf("Could not submit CMD IN transfer\n");
        exit(1);
    }
}


static void data_in_cb(struct libusb_transfer *transfer)
{
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		fprintf(stderr, "DATA IN transfer status %d\n", transfer->status);
        exit(1);
	}

	printf("DATA IN transfer callback\n");
    print_buffer(transfer->buffer, transfer->actual_length);

	if (libusb_submit_transfer(data_in_transfer) < 0) {
        printf("Could not submit DATA IN transfer\n");
        exit(1);
    }
}

int main(int argc, char **argv)
{
    int r;

    r = libusb_init(NULL);
    if (r < 0) {
        printf("Could not init libUSB: %d\n", r);
        return -1;
    }
    libusb_set_debug(NULL, 10);

    printf("Waiting for device\n");
    do {
        devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID_MICROCHIP, PRODUCT_ID_USB_FIFO);
    } while (!devh);

    r = libusb_claim_interface(devh, 0);
    if (r < 0) {
        printf("Could not claim interface: %d\n", r);
        goto out;
    }

    cmd_in_transfer = libusb_alloc_transfer(0);
    if (!cmd_in_transfer) {
        printf("Could not alloc CMD IN transfer\n");
        goto out;
    }
    libusb_fill_bulk_transfer(cmd_in_transfer, devh, EP_CMD_BULK_IN, cmd_in_buf,
            sizeof(cmd_in_buf), cmd_in_cb, NULL, 0);

    data_in_transfer = libusb_alloc_transfer(0);
    if (!data_in_transfer) {
        printf("Could not alloc DATA IN transfer\n");
        goto out;
    }
    libusb_fill_bulk_transfer(data_in_transfer, devh, EP_DATA_BULK_IN, data_in_buf,
            sizeof(data_in_buf), data_in_cb, NULL, 0);

	r = libusb_submit_transfer(cmd_in_transfer);
	if (r < 0) {
        printf("Could not submit CMD IN transfer: %d\n", r);
        goto out;
    }

	r = libusb_submit_transfer(data_in_transfer);
	if (r < 0) {
        printf("Could not submit DATA IN transfer: %d\n", r);
		/* libusb_cancel_transfer(cmd_in_transfer); */
        goto out;
	}

    {
        unsigned char command = 0xAA;
        struct libusb_transfer *cmd_out_transfer;
        cmd_out_transfer = libusb_alloc_transfer(0);
        if (!cmd_out_transfer) {
            printf("Could not alloc CMD OUT transfer\n");
            goto out;
        }
        libusb_fill_bulk_transfer(cmd_out_transfer, devh, EP_CMD_BULK_OUT, &command,
                sizeof(command), NULL, NULL, 0);
        r = libusb_submit_transfer(cmd_out_transfer);
        if (r < 0) {
            printf("Could not submit CMD OUT transfer: %d\n", r);
            goto out;
        }
    }

    do {
		r = libusb_handle_events(NULL);
		if (r < 0) {
            printf("Could not handle libUSB events: %d\n", r);
        }
    } while (r == 0);

  out:
    if (cmd_in_transfer)
        libusb_free_transfer(cmd_in_transfer);
    if (data_in_transfer)
        libusb_free_transfer(data_in_transfer);
    if (devh) {
        libusb_release_interface(devh, 0);
        libusb_close(devh);
    }
    libusb_exit(NULL);

    return 0;
}
