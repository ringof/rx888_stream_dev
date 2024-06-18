/*

Copyright (c) 2021 Ruslan Migirov <trapi78@gmail.com>
Copyright (c) 2024 David Goncalves <dave@w1euj.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#include "ezusb.h"
#include <errno.h>
#include <getopt.h>
#include <libusb.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

unsigned int queuedepth = 16; // Number of requests to queue
unsigned int reqsize = 8;     // Request size in number of packets
unsigned int duration = 100;  // Duration of the test in seconds

const char *firmware = NULL;

static unsigned int ep = 1 | LIBUSB_ENDPOINT_IN;

static int interface_number = 0;
static struct libusb_device_handle *dev_handle = NULL;
unsigned int pktsize;
unsigned int success_count = 0;  // Number of successful transfers
unsigned int failure_count = 0;  // Number of failed transfers
unsigned int transfer_size = 0;  // Size of data transfers performed so far
unsigned int transfer_index = 0; // Write index into the transfer_size array
volatile bool stop_transfers = false; // Request to stop data transfers
volatile int xfers_in_progress = 0;

volatile int sleep_time = 0;

int verbose;
static int randomizer;
static int dither;
static int has_firmware;
static int refclock_10M;

static void transfer_callback(struct libusb_transfer *transfer) {
    int size = 0;
    int ret = 0;

    xfers_in_progress--;

    if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
        failure_count++;
        fprintf(stderr, "Transfer callback status %s received %d \
	   bytes.\n",
                libusb_error_name(transfer->status), transfer->actual_length);
    } else {
        size = transfer->actual_length;
        success_count++;
        uint16_t *samples = (uint16_t *)transfer->buffer;
        if (randomizer) {
            for (int i = 0; i < size / 2; i++) {
                samples[i] ^= 0xfffe * (samples[i] & 1);
            }
        }
        ret = write(STDOUT_FILENO, transfer->buffer, transfer->actual_length);
        if (ret < 0) {
            fprintf(stderr, "Error writing to stdout: %s", strerror(errno));
        }
    }
    if (!stop_transfers) {
        if (libusb_submit_transfer(transfer) == 0)
            xfers_in_progress++;
    }
}

// Function to free data buffers and transfer structures
static void free_transfer_buffers(unsigned char **databuffers,
                                  struct libusb_transfer **transfers) {
    // Free up any allocated data buffers
    if (databuffers != NULL) {
        for (unsigned int i = 0; i < queuedepth; i++) {
            if (databuffers[i] != NULL) {
                free(databuffers[i]);
            }
            databuffers[i] = NULL;
        }
        free(databuffers);
    }

    // Free up any allocated transfer structures
    if (transfers != NULL) {
        for (unsigned int i = 0; i < queuedepth; i++) {
            if (transfers[i] != NULL) {
                libusb_free_transfer(transfers[i]);
            }
            transfers[i] = NULL;
        }
        free(transfers);
    }
}

static void sig_stop(int signum) {

    (void)signum;
    fprintf(stderr, "\nAbort. Stopping transfers\n");
    stop_transfers = true;
}
static void printhelp(void) {
    fprintf(stderr, " --verbose, -v      Verbose output\n");
    fprintf(stderr, " --firmware, -f     Firmware file\n");
    fprintf(stderr, " --dither, -d       Enable dithering\n");
    fprintf(stderr, " --rand, -r         Enable output randomization\n");
    fprintf(stderr, " --samplerate, -s   Sample Rate, default 32000000\n");
    fprintf(stderr, " --gainmode, -m     Gain Mode low/high, default high\n");
    fprintf(stderr, " --att, -a          Attenuation, default 0\n");
    fprintf(stderr, " --gain, -g         Gain value, default 0\n");
    fprintf(stderr, " --queuedepth, -q   Queue depth, default 16\n");
    fprintf(stderr,
            " --reqsize, -p      Packets per transfer request, default 8\n");
    fprintf(stderr, " --refclock-10M, -T  use 10 MHz refclock (27 MHz default)\n");
    fprintf(stderr, " --help, -h         Print this help\n");
}

// Determine actual (vs requested) clock frequency
// Adapted from code by Franco Venturi, K4VZ

static uint32_t xtalFreq = 27000000;

static double actual_freq(double frequency){
    while (frequency < 1000000)
        frequency = frequency * 2;

    // Calculate the division ratio. 900,000,000 is the maximum internal
    // PLL frequency: 900MHz
    uint32_t divider = 900000000UL / frequency;
    // Ensure an even integer division ratio
    if (divider % 2) divider--;

    // Calculate the pllFrequency: the divider * desired output frequency
    uint32_t pllFreq = divider * frequency;
#if 1
    fprintf(stderr, "pllA Freq %d\n", pllFreq);
#endif

    // Determine the multiplier to get to the required pllFrequency
    uint8_t mult = pllFreq / xtalFreq;
    // It has three parts:
    //    mult is an integer that must be in the range 15..90
    //    num and denom are the fractional parts, the numerator and denominator
    //    each is 20 bits (range 0..1048575)
    //    the actual multiplier is  mult + num / denom
    uint32_t l = pllFreq % xtalFreq;
    double f = (double)l;
    f *= 1048575;
    f /= xtalFreq;
    uint32_t num = (uint32_t)f;
    // For simplicity we set the denominator to the maximum 1048575
    uint32_t denom = 1048575;

    double actualPllFreq = (double) xtalFreq * (mult + (double) num / (double) denom);
#if 1
    fprintf(stderr, "actual PLL frequency: %d * (%d + %d / %d) = %lf\n", xtalFreq, mult, num, denom,actualPllFreq);
#endif

    double actualAdcFreq = actualPllFreq / (double) divider;
#if 1
    fprintf(stderr, "actual ADC frequency: %lf / %d = %lf\n", actualPllFreq, divider, actualAdcFreq);
#endif
    return actualAdcFreq;
}

int main(int argc, char **argv) {

    unsigned int samplerate = 32000000;
    unsigned int gain = 0x80;
    unsigned int att = 0;

    int c;
    while (1) {
        static struct option long_options[] = {
            {"verbose", optional_argument, &verbose, 1},
            {"firmware", required_argument, &has_firmware, 'f'},
            {"dither", no_argument, &dither, 'd'},
            {"rand", no_argument, &randomizer, 'r'},
            {"samplerate", required_argument, 0, 's'},
            {"gainmode", required_argument, 0, 'm'},
            {"gain", required_argument, 0, 'g'},
            {"att", required_argument, 0, 'a'},
            {"queuedepth", required_argument, 0, 'q'},
            {"reqsize", required_argument, 0, 'p'},
	    {"refclock-10M", no_argument, &refclock_10M, 'T'}, 
            {"help", no_argument, 0, 'h'},
            {0, 0, 0, 0}};

        int option_index = 0;
        int gainvalue = 0;

        c = getopt_long(argc, argv, "f:drs:hm:g:a:q:p:T", long_options,
                        &option_index);

        if (c == -1)
            break;

        if (c == 0) {
            if (long_options[option_index].flag != 0)
                break;
            c = long_options[option_index].val;
        }
        switch (c) {

        case 'f':
            firmware = optarg;
            break;

        case 'r':
            randomizer = 1;
            break;

        case 'd':
            dither = 1;
            break;

        case 'v':
            if (optarg) {
                verbose = strtoul(optarg, NULL, 10);
            } else {
                verbose = 1;
            }
            break;

        case 's':
            samplerate = strtoul(optarg, NULL, 10);
            if (samplerate < 1000000) {
                fprintf(stderr, "Invalid samplerate %d\n", samplerate);
                printhelp();
                return 0;
            }
            break;

        case 'm':
            if (strcmp(optarg, "high") == 0) {
                gain |= 0x80;
            } else if (strcmp(optarg, "low") == 0) {
                gain &= ~0x80;
            } else {
                fprintf(stderr, "Invalid gain mode %s\n", optarg);
                printhelp();
                return 0;
            }
            break;

        case 'g':
            gainvalue = strtol(optarg, NULL, 10);
            if (gainvalue < 0 || gainvalue > 127) {
                fprintf(stderr, "Invalid gain value %d\n", gainvalue);
                printhelp();
                return 0;
            }
            gain &= ~0x7f;
            gain |= gainvalue;
            break;

        case 'a':
            att = strtol(optarg, NULL, 10);
            if (att < 0 || att > 63) {
                fprintf(stderr, "Invalid attenuation value %d\n", att);
                printhelp();
                return 0;
            }
            break;
        case 'q':
            queuedepth = strtol(optarg, NULL, 10);
            if (queuedepth < 1 || queuedepth > 64) {
                fprintf(stderr, "Invalid queue depth %d\n", queuedepth);
                printhelp();
                return 0;
            }
            break;
        case 'p':
            reqsize = strtol(optarg, NULL, 10);
            if (reqsize < 1 || reqsize > 64) {
                fprintf(stderr, "Invalid request size %d\n", reqsize);
                printhelp();
                return 0;
            }
            break;
        case 'T':
            xtalFreq = (uint32_t)10000000;
            break;
        case 'h':
        case '?':
        default:
            /* getopt_long already printed an error message. */
            printhelp();
            return 0;
        }
    }

    fprintf(stderr, "Firmware: %s\n", firmware);
    fprintf(stderr, "Ref. Clock: %d\n", xtalFreq);
    fprintf(stderr, "Requested Sample Rate: %u\n", samplerate);
    actual_freq((double)samplerate);
    fprintf(stderr, "Output Randomizer %s, Dither: %s\n",
            randomizer ? "On" : "Off", dither ? "On" : "Off");
    fprintf(stderr, "Gain Mode: %s, Gain: %u, Att: %u\n",
            (gain & 0x80) ? "High" : "Low", gain & 0x7f, att);
    /* code */
    struct libusb_device_descriptor desc;
    struct libusb_device *dev;
    struct libusb_endpoint_descriptor const *endpointDesc;
    struct libusb_ss_endpoint_companion_descriptor *ep_comp;
    struct libusb_config_descriptor *config;
    struct libusb_interface_descriptor const *interfaceDesc;
    size_t list, i;
    int ret, temp;
    int rStatus;
    struct sigaction sigact;

    sigact.sa_handler = sig_stop;
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    (void)sigaction(SIGINT, &sigact, NULL);
    (void)sigaction(SIGTERM, &sigact, NULL);
    //this is needed for using streamer with a commandline tool like `pv` for limiting file size
    (void)sigaction(SIGPIPE, &sigact, NULL);
    
    struct libusb_transfer **transfers = NULL; // List of transfer structures.
    unsigned char **databuffers = NULL;        // List of data buffers.

    ret = libusb_init(NULL);
    if (ret != 0) {
        fprintf(stderr, "Error initializing libusb: %s\n",
                libusb_error_name(ret));
        exit(1);
    }

search:
    if (firmware) { // there is argument with image file
        libusb_device **devs;
        //find the RX888 device 
        list = libusb_get_device_list(NULL, &devs);
        if(list < 0) {
            fprintf(stderr, "Error in getting device list\n");
            goto close;
        }
        
        for(i=0, temp=(int)list;i<temp;i++) {
            struct libusb_device_descriptor desc;
            ret = libusb_get_device_descriptor(devs[i], &desc);
            if(ret<0) {   
                fprintf(stderr, "unable to get device descriptor\n");
                continue;
            }
            if (desc.idVendor == (uint16_t)0x04b4) {
                // no firmware, upload it
                if (desc.idProduct == (uint16_t)0x00f3) {
                    ret = libusb_open(devs[i], &dev_handle);
                    //dev = libusb_get_device(dev_handle);
                    if (ret>0) {
                         fprintf(stderr,"Error or device could not be found: error %s", libusb_error_name(ret));
                    goto close;
                    }
                    if (ezusb_load_ram(dev_handle, firmware, FX_TYPE_FX3, IMG_TYPE_IMG,
                           1) == 0) {
                        fprintf(stderr, "Firmware updated\n");
                        libusb_close(dev_handle);
                        sleep(3);
                        goto search;
                    } 
                    else {
                        fprintf(stderr, "Firmware upload failed for device\n");
                    }
                }
		        if (desc.idProduct == (uint16_t)0x00f1) {
                    libusb_open(devs[i], &dev_handle);
                    if (!dev_handle) {
                        fprintf(stderr,
                                "Error or device could not be found, try loading firmware\n");
                        goto close;
                    }
                    ret = libusb_kernel_driver_active(dev_handle, 0);
                    if (ret != 0) {
                        fprintf(stderr,
                                "Kernel driver active. Trying to detach kernel driver\n");
                        ret = libusb_detach_kernel_driver(dev_handle, 0);
                        if (ret != 0) {
                            fprintf(stderr,
                                    "Could not detach kernel driver from an interface\n");
                            goto close;
                        }
                    }
                }
            }
        }

    sleep(1);
    //this shouldn't be needed, cleanup
    dev = libusb_get_device(dev_handle);
    libusb_get_config_descriptor(dev, 0, &config);

    ret = libusb_claim_interface(dev_handle, interface_number);
    if (ret != 0) {
        fprintf(stderr, "Error claiming interface, error: %s\n", libusb_error_name(ret));
        goto end;
    }

    fprintf(stderr, "Successfully claimed interface\n");

    interfaceDesc = &(config->interface[0].altsetting[0]);

    endpointDesc = &interfaceDesc->endpoint[0];

    libusb_get_device_descriptor(dev, &desc);

    libusb_get_ss_endpoint_companion_descriptor(NULL, endpointDesc, &ep_comp);

    pktsize = endpointDesc->wMaxPacketSize * (ep_comp->bMaxBurst + 1);

    libusb_free_ss_endpoint_companion_descriptor(ep_comp);

    databuffers = (unsigned char **)calloc(queuedepth, sizeof(u_char *));
    if (databuffers == NULL) {
        fprintf(stderr, "Could not allocate memory for data buffers\n");
        goto end;
    }

    transfers = (struct libusb_transfer **)calloc(
        queuedepth, sizeof(struct libusb_transfer *));
    if (transfers == NULL) {
        fprintf(stderr, "Could not allocate memory for transfer structures\n");
        goto free_transfer_buf;
    }

    fprintf(stderr, "Queue depth: %d, Request size: %d\n", queuedepth,
            reqsize * pktsize);

    for (unsigned int i = 0; i < queuedepth; i++) {
        databuffers[i] = (unsigned char *)malloc(reqsize * pktsize);
        transfers[i] = libusb_alloc_transfer(0);
        if ((databuffers[i] == NULL) || (transfers[i] == NULL)) {
            goto free_transfer_buf;
        }
    }

    for (unsigned int i = 0; i < queuedepth; i++) {
        libusb_fill_bulk_transfer(transfers[i], dev_handle, ep, databuffers[i],
                                  reqsize * pktsize, transfer_callback,
                                  (void *)&pktsize, 0);
        rStatus = libusb_submit_transfer(transfers[i]);
        if (rStatus == 0)
            xfers_in_progress++;
    }

    /******/
    uint32_t gpio = 0;
    if (dither) {
        gpio |= DITH;
    }
    if (randomizer) {
        gpio |= RANDO;
    }

    usleep(5000);
    command_send(dev_handle, GPIOFX3, gpio);
    usleep(5000);
    argument_send(dev_handle, DAT31_ATT, att);
    usleep(5000);
    argument_send(dev_handle, AD8340_VGA, gain);
    usleep(5000);
    command_send(dev_handle, STARTADC, samplerate);
    usleep(5000);
    command_send(dev_handle, STARTFX3, 0);
    usleep(5000);
    command_send(dev_handle, TUNERSTDBY, 0);
    /*******/

    do {
        libusb_handle_events(NULL);

    } while (stop_transfers != true);

    fprintf(stderr, "Test complete. Stopping transfers\n");
    stop_transfers = true;

    while (xfers_in_progress != 0) {
        fprintf(stderr, "%d transfers are pending\n", xfers_in_progress);
        libusb_handle_events(NULL);
        usleep(100000);
    }

    fprintf(stderr, "\nTransfers completed\n");
    command_send(dev_handle, STOPFX3, 0);


free_transfer_buf:
    free_transfer_buffers(databuffers, transfers);

end:
    if (dev_handle) {
        libusb_release_interface(dev_handle, interface_number);
    }

    if (config) {
        libusb_free_config_descriptor(config);
    }

close:
    libusb_free_device_list(devs, 1);

    if (dev_handle) {
        libusb_close(dev_handle);
    }
    libusb_exit(NULL);

    return 0;
    
    }
}
