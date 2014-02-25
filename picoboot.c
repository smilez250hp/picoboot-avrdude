/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2009 Lars Immisch
 * Copyright (C) 2014 Ralph Doncaster - firstname.lastname@gmail.com
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

/* $Id: picoboot.c 1107 2012-11-20 14:03:50Z joerg_wunsch $ */

/*
 * avrdude interface for picoboot programmer
 * see http://code.google.com/p/picoboot/  
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "avrdude.h"
#include "pgm.h"
#include "serial.h"
#include "picoboot.h"

struct frame{
  uint8_t data_lo;
  uint8_t data_hi;
  uint8_t check;
  uint8_t command;
};

/* due to ACK buffering, keep MAX_FRAMES <= serial fifo size */
#define MAX_FRAMES 8

/* 2 byte virtual reset vector + 64 bytes code = 66 */
#define BOOTLOADER_SIZE 66

#define DEBUG(...) if (verbose > 1) fprintf(stderr, __VA_ARGS__)

static void picoboot_not_implemented_1 (PROGRAMMER * pgm)
{
  DEBUG("PICOBOOT: picoboot_not_implemented_1()\n");
}

static int picoboot_not_implemented_2 (PROGRAMMER * pgm, AVRPART * p)
{
  DEBUG("PICOBOOT: picoboot_not_implemented_2()\n");
  return 0;
}

void picoboot_send_frame(union filedescriptor *fd, struct frame* f)
{
  char *buf = (char*)f;

  DEBUG("PICOBOOT: picoboot_send_frame()\n");
  f->check = f->data_lo ^ f->data_hi ^ f->command;
  serial_send(fd, buf, sizeof(*f));
}

int picoboot_wait_ack(union filedescriptor *fd)
{
  char resp;

  if (serial_recv(fd, &resp, 1) < 0){
    DEBUG("PICOBOOT: picoboot_wait_ack() response not received\n");
    return -1;
  }
  else if (resp != 0) {
    fprintf(stderr,
      "\n%s: picoboot_wait_ack(): protocol error, "
      "expect ACK=0x%02x, resp=0x%02x\n",
      progname, 0, resp);
    exit(1);
  }

  return 0;
}

/* send multiple frames back-to-back for higher serial throughput */
int picoboot_buffered_send(union filedescriptor *fd, struct frame* f)
{
  static char buf[MAX_FRAMES * sizeof(*f)];
  static char* bufp = buf;
  int ack_count;

  f->check = f->data_lo ^ f->data_hi ^ f->command;
  memcpy (bufp, f, sizeof(*f));
  bufp += sizeof(*f);

  /* check for full buffer */
  if ( bufp - buf == MAX_FRAMES * sizeof(*f)){
    serial_send(fd, buf, bufp - buf);
    bufp = buf;
    for (ack_count = MAX_FRAMES; ack_count--;)
      if (picoboot_wait_ack(fd) != 0) return -1;
  }
  return 0;
}

static int picoboot_open(PROGRAMMER * pgm, char * port)
{
  DEBUG("PICOBOOT: picoboot_open()\n");
  strcpy(pgm->port, port);
  if (serial_open(port, pgm->baudrate? pgm->baudrate: 460800, &pgm->fd)==-1) {
    return -1;
  }

  /* Clear DTR and RTS to unload the RESET capacitor 
   * (for example in Arduino) */
  /* serial_set_dtr_rts(&pgm->fd, 0);
  usleep(250*1000); */
  /* Set DTR and RTS back to high */
  /* serial_set_dtr_rts(&pgm->fd, 1);
  usleep(50*1000); */

  return serial_drain(&pgm->fd, 1);
}

static int picoboot_initialize (PROGRAMMER * pgm, AVRPART * p)
{
  DEBUG("PICOBOOT: picoboot_initialize()\n");

  struct frame f;
  memset (&f, 0, sizeof(f));
  /* send frame of zeros */
  picoboot_send_frame(&pgm->fd, &f);
  return picoboot_wait_ack(&pgm->fd);
}

static int picoboot_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
  DEBUG("\nPICOBOOT: picoboot_read_sig_bytes()\n");

  /* bootloader doesn't support signature read, so fake it */
  m->buf[0] = 0x1e;
  m->buf[1] = 0x2a;
  m->buf[2] = 0x00;
  p->signature[0] = 0x1e;
  p->signature[1] = 0x2a;
  p->signature[2] = 0x00;
  return 0;
}

/* write partial page */
int write_bytes (unsigned int addr, unsigned int num_bytes)
{
  return 0;
}

/* write one page - called from avr.c:avr_write
 * last param num_bytes always == page_size */
static int picoboot_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                              unsigned int page_size,
                              unsigned int addr, unsigned int num_bytes)
{
  const uint8_t reset_vec_lo = 0xdf;   /* rjmp BootStart */
  const uint8_t reset_vec_hi = 0xcf;   /* rjmp BootStart */
  uint16_t appstart, vrst_vec_addr =
    m->size - BOOTLOADER_SIZE;
  struct frame f;
  union filedescriptor *fd = &pgm->fd;
 
  int fill_page_buf (uint16_t page_addr)
  {
    uint16_t cur_addr = page_addr;
    DEBUG("\nPICOBOOT: fill_page_buf() address 0x%04X\n", cur_addr);

    for (;cur_addr < (page_addr + page_size); cur_addr+=2 ){
      f.data_lo = m->buf[cur_addr];
      f.data_hi = m->buf[cur_addr+1];
      f.command = 0;
      picoboot_buffered_send(fd, &f);

      f.data_lo = cur_addr & 0xff;
      f.data_hi = (cur_addr & 0xff00) >> 8;
      f.command = 0x01; /* fill temp buffer */
      picoboot_buffered_send(fd, &f);
    }
    return 0;
  }

  int erase_page (int page_addr)
  {
    f.data_lo = page_addr & 0xff;
    f.data_hi = (page_addr & 0xff00) >> 8;
    f.command = 0x03; /* erase page */
    picoboot_send_frame(fd, &f);
    if (picoboot_wait_ack(fd) != 0) return -1;
    return 0;
  }

  int write_page (int page_addr)
  {
    f.data_lo = page_addr & 0xff;
    f.data_hi = (page_addr & 0xff00) >> 8;
    f.command = 0x05; /* write page */
    picoboot_send_frame(fd, &f);
    if (picoboot_wait_ack(fd) != 0) return -1;
    return 0;
  }

  DEBUG("\nPICOBOOT: picoboot_paged_write() address 0x%04X\n", addr);

  /* only flash write supported */
  if (strcmp(m->desc, "flash") != 0){
    DEBUG("\nPICOBOOT: no support for writing %s.\n", m->desc);
    return -1;
  }

  if ( addr >= vrst_vec_addr) {
    fprintf(stderr,
      "\n%s: picoboot_paged_write(): error, "
      "attempt to write to bootloader memory.\n",
      progname);
    exit (1);
  }

  if ( addr > (vrst_vec_addr - page_size)) {
    /* virtual reset vector page written along with page 0 */
    return num_bytes;
  }

  if ( addr == 0 ) {
    uint16_t vrst_vec_page = vrst_vec_addr - page_size +2;

    /* save and redirect reset vector */
    appstart = *((uint8_t *)m->buf) | 
               *((uint8_t *)m->buf +1 ) << 8; 
    /* validate rjmp at reset vector */
    if ((appstart & 0xF000) != 0xC000) {
      fprintf(stderr,
        "\n%s: picoboot_paged_write(): error, "
        "No reset vector in flash file.\n",
        progname);
      exit (1);
    } 

    *((uint8_t *)m->buf) = reset_vec_lo;
    *((uint8_t *)m->buf+1) = reset_vec_hi;

    /* calculate new rjmp for appstart */
    appstart = 0xc000 |
               (((appstart & 0x0FFF) + BOOTLOADER_SIZE/2) & 0x0FFF);
    m->buf[vrst_vec_addr] = appstart & 0x00FF;
    m->buf[vrst_vec_addr+1] = appstart >> 8;
    DEBUG("\nPICOBOOT: virtual reset vector 0x%04x at 0x%04x.\n",
          appstart, vrst_vec_addr);

    if (fill_page_buf(vrst_vec_page) != 0) return -1;
    if (erase_page(vrst_vec_page) != 0) return -1;
    if (write_page(vrst_vec_page) != 0) return -1;
  }
 
  if (fill_page_buf(addr) != 0) return -1;
  if (erase_page(addr) != 0) return -1;
  if (write_page(addr) != 0) return -1;

  return num_bytes;
}

static void picoboot_close(PROGRAMMER * pgm)
{
  DEBUG("PICOBOOT: picoboot_close()\n");
  serial_close(&pgm->fd);
  pgm->fd.ifd = -1;
}

const char picoboot_desc[] = "picoboot bootloader";

void picoboot_initpgm(PROGRAMMER * pgm)
{
  DEBUG("PICOBOOT: picoboot_initpgm()\n");

  strcpy(pgm->type, "Picoboot");
  pgm->open = picoboot_open;
  pgm->enable = picoboot_not_implemented_1;
  pgm->initialize = picoboot_initialize;
  pgm->read_sig_bytes = picoboot_read_sig_bytes;
  pgm->program_enable = picoboot_not_implemented_2;
  pgm->chip_erase     = picoboot_not_implemented_2;
  pgm->paged_write    = picoboot_paged_write;
  pgm->close = picoboot_close;
}
