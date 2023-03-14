/**
 * @file   usbhd44780u.c
 * @author Ingo A. Kubbilun (ingo.kubbilun@gmail.com)
 * @brief  implementation of a convenience library that communicates
 *         with the also supplied Linux kernel driver ch341ahd44780u.ko:
 *         USB bridge controller CH341A connected to a HITACHI HD44780U
 *         LCD display with four rows and 20 columns.
 *
 * [MIT license]
 *
 * Copyright (c) 2023 Ingo A. Kubbilun
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <usbhd44780u.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>

#define likely(x)                     __builtin_expect((x),1)
#define unlikely(x)                   __builtin_expect((x),0)

#define DISP_FLAG_ON                  0x00000001
#define DISP_FLAG_CURSOR              0x00000002
#define DISP_FLAG_BLINKING            0x00000004
#define DISP_FLAG_AUTOSCROLL          0x00000008

struct _ch341a_hd44780u
{
  uint32_t          devno;            /* zero-based device number */
  int               fd;               /* the file descriptor */

  uint32_t          x;                /* current cursor x position (column): 0..19 */
  uint32_t          y;                /* current cursor y position (row): 0..3 */

  uint8_t           buffer[24 * 4]    /* current LCD display content; 24 bytes per row for alignment purposes */
  __attribute__((aligned(16)));

  uint32_t          flags;            /* see DISP_FLAG_xxx */

  uint32_t          ddram_addr;
};

/* the German 'umlauts' Ä Ö Ü ä ö ü ß as dot matrices for the CGRAM */
static const uint8_t dotmatrix_german_umlaut_A[8] = { 17,14,17,17,31,17,17, 0 };
static const uint8_t dotmatrix_german_umlaut_O[8] = { 17,14,17,17,17,17,14, 0 };
static const uint8_t dotmatrix_german_umlaut_U[8] = { 17, 0,17,17,17,17,14, 0 };
static const uint8_t dotmatrix_german_umlaut_a[8] = { 17, 0,14, 1,15,17,15, 0 };
static const uint8_t dotmatrix_german_umlaut_o[8] = { 17, 0,14,17,17,17,14, 0 };
static const uint8_t dotmatrix_german_umlaut_u[8] = { 17, 0,17,17,17,19,13, 0 };
static const uint8_t dotmatrix_german_umlaut_s[8] = { 12,18,18,30,17,17,30,16 };

p_ch341a_hd44780u ch341a_hd44780u_open ( uint32_t device_number )
{
  int               fd;
  char              device[128];
  p_ch341a_hd44780u p_display;

  snprintf(device,sizeof(device),"/dev/hd44780u_dp%u",device_number);

  fd = open(device,O_RDWR);
  if (unlikely(-1 == fd))
    return NULL;

  p_display = (p_ch341a_hd44780u)malloc(sizeof(ch341a_hd44780u));
  if (unlikely(NULL == p_display))
  {
    close(fd);
    return NULL;
  }
  memset(p_display, 0, sizeof(ch341a_hd44780u));

  p_display->devno = device_number;
  p_display->fd = fd;

  memset(p_display->buffer, 0x20, sizeof(p_display->buffer)); /* empty LCD display means: filled with spaces */

  return p_display;
}

void ch341a_hd44780u_close ( p_ch341a_hd44780u p_display )
{
  if (likely(NULL != p_display))
  {
    close(p_display->fd);
    free(p_display);
  }
}

int ch341a_hd44780u_configure ( p_ch341a_hd44780u p_display, bool on, bool cursor, bool blinking )
{
  uint32_t        old_flags, new_flags;
  uint64_t        verb;

  if (unlikely(NULL == p_display))
    return -1;

  old_flags = (p_display->flags & DISP_FLAG_ON ? 0x04 : 0x00) |
              (p_display->flags & DISP_FLAG_CURSOR ? 0x02 : 0x00) |
              (p_display->flags & DISP_FLAG_BLINKING ? 0x01 : 0x00);

  new_flags = (on ? 0x04 : 0x00) | (cursor ? 0x02: 0x00) | (blinking ? 0x01 : 0x00);

  if (old_flags == new_flags)
    return 0; /* nothing to do */

  if (on)
    p_display->flags |= DISP_FLAG_ON;
  else
    p_display->flags &= ~DISP_FLAG_ON;

  if (cursor)
    p_display->flags |= DISP_FLAG_CURSOR;
  else
    p_display->flags &= ~DISP_FLAG_CURSOR;

  if (blinking)
    p_display->flags |= DISP_FLAG_BLINKING;
  else
    p_display->flags &= ~DISP_FLAG_BLINKING;

  verb = 0x020000000000A608 | ((uint64_t)new_flags);

  return ioctl(p_display->fd, 0x80000000, verb);
}

int ch341a_hd44780u_clear ( p_ch341a_hd44780u p_display )
{
  int retval;

  if (unlikely(NULL == p_display))
    return -1;

  retval = ioctl(p_display->fd, 0x80000000, 0x020000000000A601);

  if (0 == retval)
  {
    p_display->x = p_display->y = p_display->ddram_addr = 0;
    memset(p_display->buffer, 0x20, sizeof(p_display->buffer));
  }

  return retval;
}

/**
 * @brief converts a cursor position (x,y) into a DDRAM address
 *
 * The first row is (0x00..0x13), the second row is (0x40..0x53),
 * the third row is (0x14..0x27), and the fourth row is (0x54..0x67).
 *
 * @param[in] x   column / x-coordinate 0..19
 * @param[in] y   row / y-coordinate 0..3
 */
static uint8_t pos2addr ( uint32_t x, uint32_t y )
{
  static const uint8_t ddram_addrs[80] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13,
                                           0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53,
                                           0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
                                           0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67 };

  return ddram_addrs[ (y & 3) * 20 + (x % 20) ];
}

int ch341a_hd44780u_setcursorpos ( p_ch341a_hd44780u p_display, uint32_t x, uint32_t y )
{
  uint8_t ddram_addr;
  int retval;

  if (unlikely(NULL == p_display || x > 19 || y > 3))
    return -1;

  ddram_addr = pos2addr(x,y);

  retval = ioctl(p_display->fd, 0x80000000, 0x020000000000A680 | ddram_addr);

  if (0 == retval)
  {
    p_display->ddram_addr = ddram_addr;
    p_display->x = x;
    p_display->y = y;
  }

  return retval;
}

int ch341a_hd44780u_getcursorpos ( p_ch341a_hd44780u p_display, uint32_t *p_x, uint32_t *p_y )
{
  if (unlikely(NULL == p_display || NULL == p_x || NULL == p_y))
    return -1;

  *p_x = p_display->x;
  *p_y = p_display->y;

  return 0;
}

int ch341a_hd44780u_autoscroll ( p_ch341a_hd44780u p_display, bool autoscroll )
{
  if (unlikely(NULL == p_display))
    return -1;

  if (autoscroll)
    p_display->flags |= DISP_FLAG_AUTOSCROLL;
  else
    p_display->flags &= ~DISP_FLAG_AUTOSCROLL;

  return 0;
}

static int perform_delta_display_op ( p_ch341a_hd44780u p_display, const uint8_t *p_buffer, bool force_ddram_addr_update )
{
  uint32_t          idx = 0, ddram_addr, ddram_addr_saved, ddram_addr_cur, x = 0, y = 0;
  uint8_t           cmds[512] __attribute__((aligned(16)));
  int               retval;

  ddram_addr_saved = ddram_addr_cur = p_display->ddram_addr;

  // if cursor shown, hide it

  if (p_display->flags & DISP_FLAG_CURSOR) // cursor shown? Then hide it first
  {
    cmds[idx++] = 0x02;
    cmds[idx++] = 0xA6;
    cmds[idx++] = 0x08 | (p_display->flags & DISP_FLAG_ON ? 0x04 : 0x00);
  }

  for (y = 0; y < 4; y ++)
  {
    for (x = 0; x < 20; x++)
    {
      ddram_addr = pos2addr(x,y);

      if (p_buffer[y * 24 + x] != p_display->buffer[y * 24 + x]) // this character has to be modified
      {
        if (ddram_addr != ddram_addr_cur) // program new ddram address
        {
          cmds[idx++] = 0x02;
          cmds[idx++] = 0xA6;
          cmds[idx++] = 0x80 | ((uint8_t)ddram_addr);
          ddram_addr_cur = ddram_addr;
        }

        // write new character to the position

        cmds[idx++] = 0x02;
        cmds[idx++] = 0xA7;
        cmds[idx++] = p_buffer[y * 24 + x];

        ddram_addr_cur++; // a character write increases the address in the ddram
      }
    } // of for x (column)
  } // of for y (column)

  if (force_ddram_addr_update)
  {
    p_display->ddram_addr = pos2addr(p_display->x, p_display->y);
    cmds[idx++] = 0x02;
    cmds[idx++] = 0xA6;
    cmds[idx++] = 0x80 | ((uint8_t)p_display->ddram_addr);
  }
  else
  if (ddram_addr_cur != ddram_addr_saved)
  {
    cmds[idx++] = 0x02;
    cmds[idx++] = 0xA6;
    cmds[idx++] = 0x80 | ((uint8_t)ddram_addr_saved);
  }

  if (p_display->flags & DISP_FLAG_CURSOR) // cursor shown? Then restore it
  {
    cmds[idx++] = 0x02;
    cmds[idx++] = 0xA6;
    cmds[idx++] = 0x08 | (p_display->flags & DISP_FLAG_ON ? 0x04 : 0x00) | 0x02 | (p_display->flags & DISP_FLAG_BLINKING ? 0x01 : 0x00);
  }

  retval = ioctl(p_display->fd, 0x80000000 | idx, (unsigned long)cmds); // send full sequence to LCD display

  if (0 == retval)
    memcpy(p_display->buffer, p_buffer, 24 * 4);

  return retval;
}

int ch341a_hd44780u_scroll ( p_ch341a_hd44780u p_display, uint32_t rows )
{
  uint8_t           buffer[24 * 4] __attribute__((aligned(16)));
  uint8_t           cmds[16] __attribute__((aligned(16)));
  uint32_t          idx;
  int               retval;

  if (unlikely( NULL == p_display ))
    return -1;

  switch(rows)
  {
    case 0:
      return 0;

    case 1:
      ((uint64_t*)buffer)[ 0] = ((uint64_t*)p_display->buffer)[ 3];
      ((uint64_t*)buffer)[ 1] = ((uint64_t*)p_display->buffer)[ 4];
      ((uint64_t*)buffer)[ 2] = ((uint64_t*)p_display->buffer)[ 5];

      ((uint64_t*)buffer)[ 3] = ((uint64_t*)p_display->buffer)[ 6];
      ((uint64_t*)buffer)[ 4] = ((uint64_t*)p_display->buffer)[ 7];
      ((uint64_t*)buffer)[ 5] = ((uint64_t*)p_display->buffer)[ 8];

      ((uint64_t*)buffer)[ 6] = ((uint64_t*)p_display->buffer)[ 9];
      ((uint64_t*)buffer)[ 7] = ((uint64_t*)p_display->buffer)[10];
      ((uint64_t*)buffer)[ 8] = ((uint64_t*)p_display->buffer)[11];

      ((uint64_t*)buffer)[ 9] = 0x2020202020202020;
      ((uint64_t*)buffer)[10] = 0x2020202020202020;
      ((uint64_t*)buffer)[11] = 0x2020202020202020;
      break;

    case 2:
      ((uint64_t*)buffer)[ 0] = ((uint64_t*)p_display->buffer)[ 6];
      ((uint64_t*)buffer)[ 1] = ((uint64_t*)p_display->buffer)[ 7];
      ((uint64_t*)buffer)[ 2] = ((uint64_t*)p_display->buffer)[ 8];

      ((uint64_t*)buffer)[ 3] = ((uint64_t*)p_display->buffer)[ 9];
      ((uint64_t*)buffer)[ 4] = ((uint64_t*)p_display->buffer)[10];
      ((uint64_t*)buffer)[ 5] = ((uint64_t*)p_display->buffer)[11];

      ((uint64_t*)buffer)[ 6] = 0x2020202020202020;
      ((uint64_t*)buffer)[ 7] = 0x2020202020202020;
      ((uint64_t*)buffer)[ 8] = 0x2020202020202020;

      ((uint64_t*)buffer)[ 9] = 0x2020202020202020;
      ((uint64_t*)buffer)[10] = 0x2020202020202020;
      ((uint64_t*)buffer)[11] = 0x2020202020202020;
      break;

    case 3:
      ((uint64_t*)buffer)[ 0] = ((uint64_t*)p_display->buffer)[ 9];
      ((uint64_t*)buffer)[ 1] = ((uint64_t*)p_display->buffer)[10];
      ((uint64_t*)buffer)[ 2] = ((uint64_t*)p_display->buffer)[11];

      ((uint64_t*)buffer)[ 3] = 0x2020202020202020;
      ((uint64_t*)buffer)[ 4] = 0x2020202020202020;
      ((uint64_t*)buffer)[ 5] = 0x2020202020202020;

      ((uint64_t*)buffer)[ 6] = 0x2020202020202020;
      ((uint64_t*)buffer)[ 7] = 0x2020202020202020;
      ((uint64_t*)buffer)[ 8] = 0x2020202020202020;

      ((uint64_t*)buffer)[ 9] = 0x2020202020202020;
      ((uint64_t*)buffer)[10] = 0x2020202020202020;
      ((uint64_t*)buffer)[11] = 0x2020202020202020;
      break;

    default: // clear screen but position cursor to last known position!!!
      idx = 0;

      if (p_display->flags & DISP_FLAG_CURSOR) // cursor shown? Then hide it first
      {
        cmds[idx++] = 0x02;
        cmds[idx++] = 0xA6;
        cmds[idx++] = 0x08 | (p_display->flags & DISP_FLAG_ON ? 0x04 : 0x00);
      }

      cmds[idx++] = 0x02; // clear screen
      cmds[idx++] = 0xA6;
      cmds[idx++] = 0x01;

      if (0 != p_display->ddram_addr) // move cursor = address to last known position
      {
        cmds[idx++] = 0x02;
        cmds[idx++] = 0xA6;
        cmds[idx++] = 0x80 | ((uint8_t)p_display->ddram_addr);
      }

      if (p_display->flags & DISP_FLAG_CURSOR) // cursor shown? Then restore it
      {
        cmds[idx++] = 0x02;
        cmds[idx++] = 0xA6;
        cmds[idx++] = 0x08 | (p_display->flags & DISP_FLAG_ON ? 0x04 : 0x00) | 0x02 | (p_display->flags & DISP_FLAG_BLINKING ? 0x01 : 0x00);
      }

      retval = ioctl(p_display->fd, 0x80000000 | idx, (unsigned long)cmds); // send full sequence to LCD display

      if (0 == retval)
        memset(p_display->buffer, 0x20, sizeof(p_display->buffer));

      return retval;
  }

  // we have a 'delta' op now: buffer is new buffer, p_display->buffer is old buffer

  return perform_delta_display_op(p_display, buffer, false/*no force ddram address update*/);
}

int ch341a_hd44780u_print ( p_ch341a_hd44780u p_display, const char *p_string, int32_t l_string )
{
  uint8_t           buffer[24 * 4] __attribute__((aligned(16)));
  uint32_t          i;

  if (unlikely( NULL == p_display ))
    return -1;

  if (0 == l_string)
    return 0;

  if (NULL == p_string)
    return -1;

  if (-1 == l_string)
    l_string = (int32_t)strlen(p_string);

  memcpy(buffer, p_display->buffer, 24 * 4);

  for (i = 0; i < ((uint32_t)l_string); i++)
  {
    if (0x0A == p_string[i]) // newline#
      goto NewLine;

    buffer[ p_display->y * 24 + p_display->x ] = p_string[i];

    p_display->x++;
    if (20 == p_display->x)
    {
NewLine:
      p_display->x = 0;
      p_display->y++;
      if (4 == p_display->y)
      {
        if (p_display->flags & DISP_FLAG_AUTOSCROLL)
        {
          p_display->y = 3;

          ((uint64_t*)buffer)[ 0] = ((uint64_t*)buffer)[ 3];
          ((uint64_t*)buffer)[ 1] = ((uint64_t*)buffer)[ 4];
          ((uint64_t*)buffer)[ 2] = ((uint64_t*)buffer)[ 5];

          ((uint64_t*)buffer)[ 3] = ((uint64_t*)buffer)[ 6];
          ((uint64_t*)buffer)[ 4] = ((uint64_t*)buffer)[ 7];
          ((uint64_t*)buffer)[ 5] = ((uint64_t*)buffer)[ 8];

          ((uint64_t*)buffer)[ 6] = ((uint64_t*)buffer)[ 9];
          ((uint64_t*)buffer)[ 7] = ((uint64_t*)buffer)[10];
          ((uint64_t*)buffer)[ 8] = ((uint64_t*)buffer)[11];

          ((uint64_t*)buffer)[ 9] = 0x2020202020202020;
          ((uint64_t*)buffer)[10] = 0x2020202020202020;
          ((uint64_t*)buffer)[11] = 0x2020202020202020;
        }
        else
        {
          p_display->y = 0;
        }
      }
    }
  }

  return perform_delta_display_op(p_display, buffer, true/*DO force ddram address update*/);
}

int ch341a_hd44780u_get_displaybuffer ( p_ch341a_hd44780u p_display, uint8_t *p_displaybuffer )
{

  if (unlikely( NULL == p_display || NULL == p_displaybuffer ))
    return -1;

  memcpy(p_displaybuffer +  0, &p_display->buffer[0], 20);
  memcpy(p_displaybuffer + 20, &p_display->buffer[24], 20);
  memcpy(p_displaybuffer + 40, &p_display->buffer[48], 20);
  memcpy(p_displaybuffer + 60, &p_display->buffer[72], 20);

  return 0;
}

int ch341a_hd44780u_program_cgram ( p_ch341a_hd44780u p_display, uint32_t chidx, const uint8_t *p_dotmatrix )
{
  uint32_t        i;
  uint8_t         cmds[48];

  if (unlikely( NULL == p_display || chidx > 7 || NULL == p_dotmatrix))
    return -1;

  for (i=0; i<8; i++) /* eight rows per character */
  {
    cmds[i * 6 + 0] = 0x02;
    cmds[i * 6 + 1] = 0xA6;                                 /* program CGRAM address */
    cmds[i * 6 + 2] = 0x40 | ((uint8_t)((chidx << 3) | i)); /* row 0..7 for character chidx */
    cmds[i * 6 + 3] = 0x02;
    cmds[i * 6 + 4] = 0xA7;                                 /* write the data */
    cmds[i * 6 + 5] = p_dotmatrix[i] & 0x1F;                /* only 5 LSBs !! */
  }

  return ioctl(p_display->fd, 0x80000000 | 48, (unsigned long)cmds); // send full sequence to LCD display
}

int ch341a_hd44780u_program_german_umlauts ( p_ch341a_hd44780u p_display )
{
  int             retval;

  retval = ch341a_hd44780u_program_cgram(p_display, 0, dotmatrix_german_umlaut_A);
  if (0 != retval)
    return retval;
  retval = ch341a_hd44780u_program_cgram(p_display, 1, dotmatrix_german_umlaut_O);
  if (0 != retval)
    return retval;
  retval = ch341a_hd44780u_program_cgram(p_display, 2, dotmatrix_german_umlaut_U);
  if (0 != retval)
    return retval;
  retval = ch341a_hd44780u_program_cgram(p_display, 3, dotmatrix_german_umlaut_a);
  if (0 != retval)
    return retval;
  retval = ch341a_hd44780u_program_cgram(p_display, 4, dotmatrix_german_umlaut_o);
  if (0 != retval)
    return retval;
  retval = ch341a_hd44780u_program_cgram(p_display, 5, dotmatrix_german_umlaut_u);
  if (0 != retval)
    return retval;
  retval = ch341a_hd44780u_program_cgram(p_display, 6, dotmatrix_german_umlaut_s);
  if (0 != retval)
    return retval;

  return 0;
}

