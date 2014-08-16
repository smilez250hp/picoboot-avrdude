/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2006  Thomas Fischl
 * Copyright 2007 Joerg Wunsch <j@uriah.heep.sax.de>
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* $Id: usbasp.c 1196 2013-09-02 20:22:53Z joerg_wunsch $ */

/*
 * Interface to the USBasp programmer.
 *
 * See http://www.fischl.de/usbasp/
 */
#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "pgm.h"
#include "usbasp.h"

#if defined(HAVE_LIBUSB) || defined(HAVE_LIBUSB_1_0)

#ifdef HAVE_LIBUSB_1_0
# define USE_LIBUSB_1_0
#endif

#if defined(USE_LIBUSB_1_0)
# if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#  include <libusb-1.0/libusb.h>
# else
#  include <libusb.h>
# endif
#else
# if defined(HAVE_USB_H)
#  include <usb.h>
# elif defined(HAVE_LUSB0_USB_H)
#  include <lusb0_usb.h>
# else
#  error "libusb needs either <usb.h> or <lusb0_usb.h>"
# endif
#endif

#ifdef USE_LIBUSB_1_0

static libusb_context *ctx = NULL;

static int libusb_to_errno(int result)
{
	switch (result) {
	case LIBUSB_SUCCESS:
		return 0;
	case LIBUSB_ERROR_IO:
		return EIO;
	case LIBUSB_ERROR_INVALID_PARAM:
		return EINVAL;
	case LIBUSB_ERROR_ACCESS:
		return EACCES;
	case LIBUSB_ERROR_NO_DEVICE:
		return ENXIO;
	case LIBUSB_ERROR_NOT_FOUND:
		return ENOENT;
	case LIBUSB_ERROR_BUSY:
		return EBUSY;
#ifdef ETIMEDOUT
	case LIBUSB_ERROR_TIMEOUT:
		return ETIMEDOUT;
#endif
#ifdef EOVERFLOW
	case LIBUSB_ERROR_OVERFLOW:
		return EOVERFLOW;
#endif
	case LIBUSB_ERROR_PIPE:
		return EPIPE;
	case LIBUSB_ERROR_INTERRUPTED:
		return EINTR;
	case LIBUSB_ERROR_NO_MEM:
		return ENOMEM;
	case LIBUSB_ERROR_NOT_SUPPORTED:
		return ENOSYS;
	default:
		return ERANGE;
	}
}

#endif


/*
 * Private data for this programmer.
 */
struct pdata
{
#ifdef USE_LIBUSB_1_0
  libusb_device_handle *usbhandle;
#else
  usb_dev_handle *usbhandle;
#endif
  int sckfreq_hz;
  unsigned int capabilities;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))
#define IMPORT_PDATA(pgm) struct pdata *pdata = PDATA(pgm)



/* Prototypes */
// interface - management
static void aspspi_setup(PROGRAMMER * pgm);
static void aspspi_teardown(PROGRAMMER * pgm);
// internal functions
static int aspspi_transmit(PROGRAMMER * pgm, unsigned char receive,
			   unsigned char functionid, const unsigned char *send,
			   unsigned char *buffer, int buffersize);

// interface - prog.
//static int aspspi_open(PROGRAMMER * pgm, char * port);
static void aspspi_close(PROGRAMMER * pgm);
// dummy functions
static void aspspi_disable(PROGRAMMER * pgm);
static void aspspi_enable(PROGRAMMER * pgm);
static void aspspi_display(PROGRAMMER * pgm, const char * p);
// universal functions
static int aspspi_initialize(PROGRAMMER * pgm, AVRPART * p);
// SPI specific functions
static int aspspi_spi_cmd(PROGRAMMER * pgm, const unsigned char *cmd, unsigned char *res);

static int aspspi_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
  /* do nothing */
  return 0;
}

/* Interface - management */
static void aspspi_setup(PROGRAMMER * pgm)
{
  if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0) {
    fprintf(stderr,
	    "%s: aspspi_setup(): Out of memory allocating private data\n",
	    progname);
    exit(1);
  }
  memset(pgm->cookie, 0, sizeof(struct pdata));
}

static void aspspi_teardown(PROGRAMMER * pgm)
{
  free(pgm->cookie);
}

/* Internal functions */

static const char *aspspi_get_funcname(unsigned char functionid)
{
  switch (functionid) {
  case USBASP_FUNC_CONNECT:         return "USBASP_FUNC_CONNECT";         break;
  case USBASP_FUNC_DISCONNECT:      return "USBASP_FUNC_DISCONNECT";      break;
  case USBASP_FUNC_TRANSMIT:        return "USBASP_FUNC_TRANSMIT";        break;
  case USBASP_FUNC_READFLASH:       return "USBASP_FUNC_READFLASH";       break;
  case USBASP_FUNC_ENABLEPROG:      return "USBASP_FUNC_ENABLEPROG";      break;
  case USBASP_FUNC_WRITEFLASH:      return "USBASP_FUNC_WRITEFLASH";      break;
  case USBASP_FUNC_READEEPROM:      return "USBASP_FUNC_READEEPROM";      break;
  case USBASP_FUNC_WRITEEEPROM:     return "USBASP_FUNC_WRITEEEPROM";     break;
  case USBASP_FUNC_SETLONGADDRESS:  return "USBASP_FUNC_SETLONGADDRESS";  break;
  case USBASP_FUNC_SETISPSCK:       return "USBASP_FUNC_SETISPSCK";       break;
  case USBASP_FUNC_GETCAPABILITIES: return "USBASP_FUNC_GETCAPABILITIES"; break;
  default:                          return "Unknown USBASP function";     break;
  }
}

/*
 * wrapper for usb_control_msg call
 */
static int aspspi_transmit(PROGRAMMER * pgm,
			   unsigned char receive, unsigned char functionid,
			   const unsigned char *send,
			   unsigned char *buffer, int buffersize)
{
  int nbytes;

  if (verbose > 3) {
    fprintf(stderr,
	    "%s: aspspi_transmit(\"%s\", 0x%02x, 0x%02x, 0x%02x, 0x%02x)\n",
	    progname,
	    aspspi_get_funcname(functionid), send[0], send[1], send[2], send[3]);
    if (!receive && buffersize > 0) {
      int i;
      fprintf(stderr, "%s => ", progbuf);
      for (i = 0; i < buffersize; i++)
	fprintf(stderr, "[%02x] ", buffer[i]);
      fprintf(stderr, "\n");
    }
  }

#ifdef USE_LIBUSB_1_0
  nbytes = libusb_control_transfer(PDATA(pgm)->usbhandle,
				   (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | (receive << 7)) & 0xff,
				   functionid & 0xff, 
				   ((send[1] << 8) | send[0]) & 0xffff, 
				   ((send[3] << 8) | send[2]) & 0xffff, 
				   (char *)buffer, 
				   buffersize & 0xffff,
				   5000);
  if(nbytes < 0){
    fprintf(stderr, "%s: error: aspspi_transmit: %s\n", progname, strerror(libusb_to_errno(nbytes)));
    return -1;
  }
#else
  nbytes = usb_control_msg(PDATA(pgm)->usbhandle,
			   USB_TYPE_VENDOR | USB_RECIP_DEVICE | (receive << 7),
			   functionid,
			   (send[1] << 8) | send[0],
			   (send[3] << 8) | send[2],
			   (char *)buffer, buffersize,
			   5000);
  if(nbytes < 0){
    fprintf(stderr, "%s: error: aspspi_transmit: %s\n", progname, usb_strerror());
    return -1;
  }
#endif

  if (verbose > 3 && receive && nbytes > 0) {
    int i;
    fprintf(stderr, "%s<= ", progbuf);
    for (i = 0; i < nbytes; i++)
      fprintf(stderr, "[%02x] ", buffer[i]);
    fprintf(stderr, "\n");
  }

  return nbytes;
}


static void aspspi_close(PROGRAMMER * pgm)
{
  if (verbose > 2)
    fprintf(stderr, "%s: aspspi_close()\n", progname);

  if (PDATA(pgm)->usbhandle!=NULL) {
    unsigned char temp[4] = {0, 0, 0, 0};

        aspspi_transmit(pgm, 1, USBASP_FUNC_DISCONNECT, temp, temp, sizeof(temp));

#ifdef USE_LIBUSB_1_0
    libusb_close(PDATA(pgm)->usbhandle);
#else
    usb_close(PDATA(pgm)->usbhandle);
#endif
  }
#ifdef USE_LIBUSB_1_0
  libusb_exit(ctx);
#else
  /* nothing for usb 0.1 ? */
#endif
}


/* Dummy functions */
static void aspspi_disable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}

static void aspspi_enable(PROGRAMMER * pgm)
{
  /* Do nothing. */

  return;
}

static void aspspi_display(PROGRAMMER * pgm, const char * p)
{
  return;
}

AVRPART *avr_part;
static int aspspi_dummy(PROGRAMMER * pgm, AVRPART * p)
{
  avr_part = p;
  return 0;
}


/* Universal functions: for both SPI and TPI */
static int aspspi_initialize(PROGRAMMER * pgm, AVRPART * p)
{
  unsigned char temp[4] = {0, 0, 0, 0};
  unsigned char res[4];
  IMPORT_PDATA(pgm);

  if (verbose > 2)
    fprintf(stderr, "%s: aspspi_initialize()\n", progname);

  /* get capabilities */
  if(aspspi_transmit(pgm, 1, USBASP_FUNC_GETCAPABILITIES, temp, res, sizeof(res)) == 4)
    pdata->capabilities = res[0] | ((unsigned int)res[1] << 8) | ((unsigned int)res[2] << 16) | ((unsigned int)res[3] << 24);
  else
    pdata->capabilities = 0;

    /* set sck period */
    pgm->set_sck_period(pgm, pgm->bitclock);

    /* connect to target device */
    aspspi_transmit(pgm, 1, USBASP_FUNC_CONNECT, temp, res, sizeof(res));

  /* wait, so device is ready to receive commands */
  usleep(10000);

  return 0;
}

/* SPI specific functions */
static int aspspi_spi_cmd(PROGRAMMER * pgm, const unsigned char *cmd,
                   unsigned char *res)
{
  unsigned char temp[4] = {0, 0, 0, 0};
  //memset(temp, 0, sizeof(temp));

  aspspi_initialize(pgm, avr_part);

  if (verbose > 2)
    fprintf(stderr, "%s: aspspi_spi_cmd(0x%02x, 0x%02x, 0x%02x, 0x%02x)%s",
	    progname, cmd[0], cmd[1], cmd[2], cmd[3],
	    verbose > 3? "...\n": "");

  int nbytes =
    aspspi_transmit(pgm, 1, USBASP_FUNC_TRANSMIT, cmd, res, 4);

  // disconnect to de-assert SS after command
  // aspspi_transmit(pgm, 1, USBASP_FUNC_DISCONNECT, temp, temp, sizeof(temp));

  if(nbytes != 4){
    if (verbose == 3)
      putc('\n', stderr);

    fprintf(stderr, "%s: error: wrong responds size\n",
	    progname);
    return -1;
  }
  if (verbose > 2) {
    if (verbose > 3)
      fprintf(stderr, "%s: aspspi_spi_cmd()", progname);
    fprintf(stderr, " => 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
	    res[0], res[1], res[2], res[3]);
  }

  return 0;
}

void aspspi_initpgm(PROGRAMMER * pgm)
{
  /* start with usbasp initialization */
  usbasp_initpgm(pgm);

  strcpy(pgm->type, "usbasp");

  /*
   * mandatory functions
   */

  pgm->initialize     = aspspi_dummy;
  pgm->display        = aspspi_display;
  pgm->enable         = aspspi_enable;
  pgm->disable        = aspspi_disable;
  pgm->program_enable = aspspi_dummy;
  pgm->cmd            = aspspi_spi_cmd;
/*  pgm->open           = aspspi_open; */
  pgm->close          = aspspi_close;
  pgm->read_byte      = avr_read_byte_default;
  pgm->write_byte     = avr_write_byte_default;

  /*
   * optional functions
   */

  pgm->setup          = aspspi_setup;
  pgm->teardown       = aspspi_teardown;
  // use usbasp set_sck_period
  //pgm->set_sck_period = aspspi_spi_set_sck_period;
  pgm->read_sig_bytes = aspspi_read_sig_bytes;
}


#else /* HAVE_LIBUSB */

static int aspspi_nousb_open (struct programmer_t *pgm, char * name)
{
  fprintf(stderr, "%s: error: no usb support. please compile again with libusb installed.\n",
	  progname);

  return -1;
}

void aspspi_initpgm(PROGRAMMER * pgm)
{
  strcpy(pgm->type, "usbasp");

  pgm->open           = aspspi_nousb_open;
}

#endif  /* HAVE_LIBUSB */

const char aspspi_desc[] = "ASP SPI debugger - see nerdralph";

