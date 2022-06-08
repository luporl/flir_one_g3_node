/*
 * Copyright (C) 2015-2016 Thomas <tomas123 @ EEVblog Electronics Community Forum>
 * Copyright (C) 2022 Instituto de Pesquisas Eldorado <eldorado.org.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "flirone.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <fcntl.h>
#include <libusb.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "font5x7.h"
#include "jpeglib.h"
#include "plank.h"

/* defines */

// color visible image
#define FRAME_WIDTH1    640
#define FRAME_HEIGHT1   480

// colorized thermal image
#define FRAME_WIDTH2    frame_width2
#define FRAME_HEIGHT2   frame_height2
// original width/height
#define FRAME_OWIDTH2   frame_owidth2
#define FRAME_OHEIGHT2  frame_oheight2

// max chars in line
#define MAX_CHARS2      (FRAME_WIDTH2 / 6 + (flirone_pro ? 0 : 1))

#define FONT_COLOR_DFLT 0xff

/* USB defs */
#define VENDOR_ID 0x09cb
#define PRODUCT_ID 0x1996

/* These are just to make USB requests easier to read */
#define REQ_TYPE    1
#define REQ         0xb
#define V_STOP      0
#define V_START     1
#define INDEX(i)    (i)
#define LEN(l)      (l)

// buffer for EP 0x85 chunks
#define BUF85SIZE   1048576     // size got from android app

/* global data */

struct f1_frame f1_frame;

static unsigned char *fb_proc;
static unsigned char *fb_proc2;

static unsigned char colormap[768];
static int f1_state;

static int frame_width2 = 80;
static int frame_height2 = 80;
static int frame_owidth2 = 80;
static int frame_oheight2 = 60;

static char flirone_pro = 0;
static char pal_inverse = 0;
static char pal_colors = 0;

static int FFC = 0;    // detect FFC

static struct libusb_device_handle *devh = NULL;
static unsigned buf85pointer = 0;
static unsigned char buf85[BUF85SIZE];

/* functions */

void font_write(unsigned char *fb, int x, int y, const char *string,
    unsigned char color)
{
    int rx, ry, pos;

    while (*string) {
        for (ry = 0; ry < 7; ry++) {
            for (rx = 0; rx < 5; rx++) {
                int v = (font5x7_basic[(*string & 0x7F) - CHAR_OFFSET][rx] >> (ry)) & 1;
                pos = (y + ry) * FRAME_WIDTH2 + (x + rx);
                fb[pos] = v ? color : fb[pos];  // transparent
            }
        }
        string++;
        x += 6;
    }
}

static double raw2temperature(unsigned short RAW)
{
    // mystery correction factor
    RAW *= 4;
    // calc amount of radiance of reflected objects ( Emissivity < 1 )
    double RAWrefl=PlanckR1/(PlanckR2*(exp(PlanckB/(TempReflected+273.15))-PlanckF))-PlanckO;
    // get displayed object temp max/min
    double RAWobj=(RAW-(1-Emissivity)*RAWrefl)/Emissivity;
    // calc object temperature
    return PlanckB/log(PlanckR1/(PlanckR2*(RAWobj+PlanckO))+PlanckF)-273.15;
}

static int vframe(char ep[], char EP_error[], int r, int actual_length,
    unsigned char buf[], unsigned char *colormap)
{
    time_t now1;
    char magicbyte[4] = { 0xEF, 0xBE, 0x00, 0x00 };
    uint32_t FrameSize, ThermalSize, JpgSize;
    int v, x, y, pos, disp;
    unsigned short pix[FRAME_OWIDTH2 * FRAME_OHEIGHT2];   // original Flir 16 Bit RAW
    size_t framesize2 = FRAME_WIDTH2 * FRAME_HEIGHT2 * 3; // 8x8x8 Bit
    int min = 0x10000, max = 0;
    int maxx = -1, maxy = -1;
    int delta, scale, med;
    int hw, hh;
    char st1[100];
    char st2[100];
    struct tm *loctime;
    int rc;

    now1 = time(NULL);
    if (r < 0) {
        if (strcmp(EP_error, libusb_error_name(r)) != 0) {
            strcpy(EP_error, libusb_error_name(r));
            ros_error("%s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s",
                ctime(&now1), ep, r, libusb_error_name(r));
            sleep(1);
        }
        return F1L_VFRAME_ERROR;
    }

    // reset buffer if the new chunk begins with magic bytes or the buffer size limit is exceeded
    if (strncmp((char *)buf, magicbyte, 4) == 0 ||
            buf85pointer + actual_length >= BUF85SIZE)
        buf85pointer = 0;

    memmove(buf85 + buf85pointer, buf, actual_length);
    buf85pointer += actual_length;

    if (strncmp((char *)buf85, magicbyte, 4) != 0) {
        //reset buff pointer
        buf85pointer = 0;
        ros_info("Reset buffer because of bad Magic Byte!");
        return F1L_VFRAME_ERROR;
    }

    // a quick and dirty job for gcc
    FrameSize   = buf85[ 8] + (buf85[ 9] << 8) + (buf85[10] << 16) + (buf85[11] << 24);
    ThermalSize = buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
    JpgSize     = buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);

    if (FrameSize + 28 > buf85pointer)
        // wait for next chunk
        return F1L_BUSY;

    // get a full frame, first print the status
    buf85pointer = 0;
    memset(fb_proc, 128, FRAME_WIDTH2 * FRAME_HEIGHT2);

    if (pal_colors) {
        for (y = 0; y < FRAME_HEIGHT2; ++y)
            for (x = 0; x < FRAME_WIDTH2; ++x)
                for (disp = 0; disp < 3; disp++)
                    fb_proc2[3 * y * FRAME_WIDTH2 + 3 * x + disp] =
                        colormap[3 * (y * 256 / FRAME_HEIGHT2) + disp];
        goto render;
    }

    // Make a unsigned short array from what comes from the thermal frame
    // find the max and min values of the array

    hw = FRAME_OWIDTH2 / 2;
    hh = FRAME_OHEIGHT2 / 2;

    for (y = 0; y < FRAME_OHEIGHT2; y++) {
        for (x = 0; x < FRAME_OWIDTH2; x++) {
            if (flirone_pro) {
                pos = 2 * (y * (FRAME_OWIDTH2 + 4) + x) + 32;
                if (x > hw)
                    pos += 4;
            } else {
                /*
                 * 32 - seems to be the header size
                 * +2 - for some reason 2 16-bit values must be skipped at the end
                 *      of each line
                 */
                pos = 2 * (y * (FRAME_OWIDTH2 + 2) + x) + 32;
            }

            v = buf85[pos] | buf85[pos + 1] << 8;
            pix[y * FRAME_OWIDTH2 + x] = v;

            if (v < min)
                min = v;
            if (v > max) {
                max = v;
                maxx = x;
                maxy = y;
            }
        }
    }

    assert(maxx != -1);
    assert(maxy != -1);

    // scale the data in the array
    delta = max - min;
    if (delta == 0)
        delta = 1;   // if max = min we have divide by zero
    scale = 0x10000 / delta;

    for (y = 0; y < FRAME_OHEIGHT2; y++) {
        for (x = 0; x < FRAME_OWIDTH2; x++) {
          int v = (pix[y * FRAME_OWIDTH2 + x] - min) * scale >> 8;

          // fb_proc is the gray scale frame buffer
          fb_proc[y * FRAME_OWIDTH2 + x] = v;   // unsigned char!!
        }
    }

    // calc medium of 2x2 center pixels
    med = pix[(hh - 1) * FRAME_OWIDTH2 + hw - 1] +
          pix[(hh - 1) * FRAME_OWIDTH2 + hw] +
          pix[hh * FRAME_OWIDTH2 + hw - 1] +
          pix[hh * FRAME_OWIDTH2 + hw];
    med /= 4;

    // Print temperatures and time
    loctime = localtime (&now1);

    f1_frame.temp_min = raw2temperature(min);
    f1_frame.temp_med = raw2temperature(med);
    f1_frame.temp_max = raw2temperature(max);
    f1_frame.temp_max_x = maxx;
    f1_frame.temp_max_y = maxy;

    sprintf(st1,"'C %.1f/%.1f/",
        f1_frame.temp_min, f1_frame.temp_med);
    sprintf(st2, "%.1f ", f1_frame.temp_max);
    strftime(&st2[strlen(st2)], 60, "%H:%M:%S", loctime);

    if (flirone_pro) {
        strcat(st1, st2);
        st1[MAX_CHARS2 - 1] = 0;
        font_write(fb_proc, 1, FRAME_OHEIGHT2, st1, FONT_COLOR_DFLT);
    } else {
        // Print in 2 lines for FLIR ONE G3
        st1[MAX_CHARS2 - 1] = 0;
        st2[MAX_CHARS2 - 1] = 0;
        font_write(fb_proc, 1, FRAME_OHEIGHT2 + 2, st1, FONT_COLOR_DFLT);
        font_write(fb_proc, 1, FRAME_OHEIGHT2 + 12, st2, FONT_COLOR_DFLT);
    }

    // show crosshairs, remove if required
    font_write(fb_proc, hw - 2, hh - 3, "+", FONT_COLOR_DFLT);

    maxx -= 4;
    maxy -= 4;

    if (maxx < 0)
        maxx = 0;
    if (maxy < 0)
        maxy = 0;
    if (maxx > FRAME_OWIDTH2 - 10)
        maxx = FRAME_OWIDTH2 - 10;
    if (maxy > FRAME_OHEIGHT2 - 10)
        maxy = FRAME_OHEIGHT2 - 10;

    font_write(fb_proc, FRAME_OWIDTH2 - 6, maxy, "<", FONT_COLOR_DFLT);
    font_write(fb_proc, maxx, FRAME_OHEIGHT2 - 8, "|", FONT_COLOR_DFLT);

    // build RGB image
    for (y = 0; y < FRAME_HEIGHT2; y++) {
        for (x = 0; x < FRAME_WIDTH2; x++) {
            // fb_proc is the gray scale frame buffer
            v = fb_proc[y * FRAME_OWIDTH2 + x];
            if (pal_inverse)
                v = 255 - v;

            for (disp = 0; disp < 3; disp++)
                // fb_proc2 is a 24bit RGB buffer
                fb_proc2[3 * y * FRAME_OWIDTH2 + 3 * x + disp] =
                    colormap[3 * v + disp];
        }
    }

render:
    rc = F1L_OK;

    // jpg Visual Image
    f1_frame.jpg_ptr = &buf85[28 + ThermalSize];
    f1_frame.jpg_sz = JpgSize;
    rc |= F1L_NEW_IMG_FRAME;

    if (strncmp((char *)&buf85[28 + ThermalSize + JpgSize + 17], "FFC", 3) == 0) {
        ros_info("drop FFC frame");
        FFC = 1;    // drop all FFC frames
    } else {
        if (FFC == 1) {
            ros_info("drop first frame after FFC");
            FFC = 0;  // drop first frame after FFC
        } else {
            // colorized RGB Thermal Image
            f1_frame.ir_ptr = fb_proc2;
            f1_frame.ir_sz = framesize2;
            rc |= F1L_NEW_IR_FRAME;
        }
    }

    return rc;
}

static int find_lvr_flirusb(void)
{
    devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    return devh ? 0 : -EIO;
}

static void usb_exit(void)
{
    //close the device
    if (devh) {
        libusb_reset_device(devh);
        libusb_close(devh);
    }
    libusb_exit(NULL);
}

static int usb_init(void)
{
    int r;

    r = libusb_init(NULL);
    if (r < 0) {
        ros_error("failed to initialise libusb");
        return -1;
    }

    r = find_lvr_flirusb();
    if (r < 0) {
        ros_error("Could not find/open device");
        goto out;
    }
    ros_info("Successfully find the Flir One G2/G3/Pro device");

    r = libusb_set_configuration(devh, 3);
    if (r < 0) {
       ros_error("libusb_set_configuration error %d", r);
       goto out;
    }
   ros_info("Successfully set usb configuration 3");

    // Claiming of interfaces is a purely logical operation;
    // it does not cause any requests to be sent over the bus.
    r = libusb_claim_interface(devh, 0);
    if (r < 0) {
        ros_error("libusb_claim_interface 0 error %d", r);
        goto out;
    }
    r = libusb_claim_interface(devh, 1);
    if (r < 0) {
        ros_error("libusb_claim_interface 1 error %d", r);
        goto out;
    }
    r = libusb_claim_interface(devh, 2);
    if (r < 0) {
        ros_error("libusb_claim_interface 2 error %d", r);
        goto out;
    }
    ros_info("Successfully claimed interface 0, 1, 2");
    return 0;

out:
    usb_exit();
    return -1;
}

struct f1_frame *f1_init(struct f1_cfg *cfg)
{
    FILE *fp;

    /* config */
    if (cfg->pro) {
        flirone_pro = 1;
        frame_width2 = 160;
        frame_height2 = 128;
        frame_owidth2 = 160;
        frame_oheight2 = 120;
    }

    if (cfg->pal_inv)
        pal_inverse = 1;

    if (cfg->pal_colors)
        pal_colors = 1;

    /* read */
    fp = fopen(cfg->pal_path, "rb");
    if (fp == NULL) {
        perror("failed to open palette");
        exit(1);
    }
    // read 256 rgb values
    if (fread(colormap, 1, 768, fp) != 768) {
        perror("failed to read colormap");
        exit(1);
    }
    fclose(fp);

    if (usb_init() < 0)
        return NULL;

    fb_proc = malloc(FRAME_WIDTH2 * FRAME_HEIGHT2);
    assert(fb_proc);
    fb_proc2 = malloc(FRAME_WIDTH2 * FRAME_HEIGHT2 * 3); // 8x8x8  Bit RGB buffer
    assert(fb_proc2);

    f1_state = 1;

    f1_frame.ir_width = FRAME_WIDTH2;
    f1_frame.ir_height = FRAME_HEIGHT2;
    return &f1_frame;
}

void f1_exit(void)
{
    // free memory
    free(fb_proc);      // thermal RAW
    free(fb_proc2);     // visible jpg
}

int f1_loop(void)
{
    int r = 0;
    unsigned char buf[BUF85SIZE];
    int actual_length;
    time_t now;
    char EP85_error[50] = "";
    unsigned char data[2] = { 0, 0 }; // only a bad dummy
    int timeout = 100;  // don't change timeout=100ms !!
    int rc = F1L_OK;

    switch (f1_state) {
    case 1:
        ros_info("stop interface 2 FRAME");
        r = libusb_control_transfer(devh, REQ_TYPE, REQ, V_STOP, INDEX(2),
                data, LEN(0), timeout);
        if (r < 0) {
            ros_error("Control Out error %d", r);
            return F1L_INIT1_ERROR;
        }

        ros_info("stop interface 1 FILEIO");
        r = libusb_control_transfer(devh, REQ_TYPE, REQ, V_STOP, INDEX(1),
                data, LEN(0), timeout);
        if (r < 0) {
            ros_error("Control Out error %d", r);
            return F1L_INIT1_ERROR;
        }

        ros_info("start interface 1 FILEIO");
        r = libusb_control_transfer(devh, REQ_TYPE, REQ, V_START,
                INDEX(1), data, LEN(0), timeout);
        if (r < 0) {
            ros_error("Control Out error %d", r);
            return F1L_INIT1_ERROR;
        }
        now = time(0);  // Get the system time
        ros_info(":xx %s", ctime(&now));
        f1_state = 2;
        break;

    case 2:
        ros_info("Ask for video stream, start EP 0x85:");
        r = libusb_control_transfer(devh, REQ_TYPE, REQ, V_START,
                INDEX(2), data, LEN(2), timeout * 2);
        if (r < 0) {
            ros_error("Control Out error %d", r);
            return F1L_INIT2_ERROR;
        }

        f1_state = 3;
        break;

    case 3:
        // endless loop
        // poll Frame Endpoints 0x85
        r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf),
                &actual_length, timeout);
        if (actual_length > 0)
            rc = vframe("0x85", EP85_error, r, actual_length, buf, colormap);
        break;
    }

    // poll Endpoints 0x81, 0x83
    r = libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);
    r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
    if (strcmp(libusb_error_name(r), "LIBUSB_ERROR_NO_DEVICE")==0) {
        ros_error("EP 0x83 LIBUSB_ERROR_NO_DEVICE -> reset USB");
        usb_exit();
        return F1L_USB_ERROR;
    }

    return rc;
}
