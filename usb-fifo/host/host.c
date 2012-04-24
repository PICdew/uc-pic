#include <stdio.h>
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <usb.h>

#define VERSION "0.01"

#define VENDOR_ID_MICROCHIP         0x04D8
#define PRODUCT_ID_USB_FIFO         0x0400

#define EP_BULK_IN                  0x81
#define EP_BULK_OUT                 0x01

#define BUF_LEN                     4096

#define READ_TO_MS                  1000
#define WRITE_TO_MS                 1000

const unsigned short pids[] = {
    PRODUCT_ID_USB_FIFO,
};

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(typeof(a[0])))

int main(int argc, char **argv)
{
    struct usb_bus *bus;
    usb_dev_handle *udev;
    int cnt;
    unsigned char buf[BUF_LEN];
    unsigned char command;


    usb_init();
    usb_find_busses();

    if (usb_find_devices()) {
        for (bus = usb_get_busses(); bus; bus = bus->next) {
            struct usb_device *dev;

            for (dev = bus->devices; dev; dev = dev->next) {
                if (dev->descriptor.idVendor  == VENDOR_ID_MICROCHIP) {
                    unsigned int i, j;

                    for (i=0;i<ARRAY_SIZE(pids);++i) {
                        if (pids[i] != dev->descriptor.idProduct)
                            continue;

                        udev = usb_open(dev);

                        if (udev) {
                            printf("device found!\n");

                            if (usb_set_configuration(udev, 1) < 0) {
                                printf("could not set configuration\n");
                                goto err;
                            }

                            if (usb_claim_interface(udev, 0) < 0) {
                                printf("could not claim interface\n");
                                goto err;
                            }

                            command = 0xAA;
                            printf("Sending command: 0x%02X\n", command);
                            if (2 != usb_bulk_write(udev, EP_BULK_OUT, (char *)&command, 2, WRITE_TO_MS)) {
                                printf("could not send command\n");
                                goto err;
                            }

                            cnt = usb_bulk_read(udev, EP_BULK_IN, (char *)buf, BUF_LEN, READ_TO_MS);
                            if (cnt <= 0) {
                                printf("could not get anything from device\n");
                                goto err;
                            }

                            printf("Received response: %d bytes\n", cnt);
                            printf("HEX:\n");
                            for (j=0;j<cnt;++j) {
                                printf("%02X ", (unsigned int)buf[j]);
                                if (j%16 == 0 && j != 0)
                                    printf("\n");
                            }
                            printf("\nCHAR:\n");
                            for (j=0;j<cnt;++j) {
                                printf("%c", (unsigned int)buf[j]);
                                if (j%16 == 0 && j != 0)
                                    printf("\n");
                            }
                            printf("\n");

                            usb_release_interface(udev, 0);
                            usb_close(udev);

                            return 0;
                        }
                    }
                }
            }
        }
    }

    return 0;

  err:
    usb_release_interface(udev, 0);
    usb_close(udev);
    return -1;
}
