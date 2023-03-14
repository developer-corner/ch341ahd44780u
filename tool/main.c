/**
 * @file   main.c
 * @author Ingo A. Kubbilun (ingo.kubbilun@gmail.com)
 * @brief  implementation of a command line tool (with testsuite) for the
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
#include <string.h>
#include <time.h>
#include <sys/time.h>

static int ms_sleep ( int millis )
{
  struct timespec ts;

  ts.tv_sec  = millis/1000;
  ts.tv_nsec = (millis%1000)*1000000;
  return nanosleep(&ts,NULL);
}

static int run_testsuite ( p_ch341a_hd44780u p_display, uint32_t device )
{
  char          buffer[128];
  int           retval, i;

  fprintf(stdout,"Successfully opened device with zero-based index %u.\n",device);
  fprintf(stdout,"Running testsuite\n");

  fprintf(stdout,"  Press <ENTER> to clear display (should be clear anyway).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_clear(p_display);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: clear display FAILED.\n");
DoError:
    fprintf(stderr,"------ errno=%i: %s\n",errno,strerror(errno));
    return 1;
  }
  fprintf(stdout,"  [CHECK!] clear display.\n");

  fprintf(stdout,"  Press <ENTER> to show cursor.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_configure(p_display, true, true, false);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: configure display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] cursor should be shown (not blinking).\n");

  fprintf(stdout,"  Press <ENTER> for blinking cursor.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_configure(p_display, true, true, true);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: configure display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] cursor should be shown (blinking).\n");

  fprintf(stdout,"  Press <ENTER> for 'Hello world' in 1st row.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "Hello world", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] 'Hello world' in 1st row with updated, blinking cursor.\n");

  fprintf(stdout,"  Press <ENTER> for 'Hello world' in 2nd row.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "\nHello world", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] 'Hello world' in 2nd row with updated, blinking cursor.\n");

  fprintf(stdout,"  Press <ENTER> for 'Hello world' in 3rd row.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "\nHello world", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] 'Hello world' in 3rd row with updated, blinking cursor.\n");

  fprintf(stdout,"  Press <ENTER> for 'Hello world' in 4th row.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "\nHello world", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] 'Hello world' in 4th row with updated, blinking cursor.\n");

  fprintf(stdout,"  Press <ENTER> for ' ABCDEFGHIJKLMNOPQRSTUVWXYZ' printing (no autoscroll!).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, " ABCDEFGHIJKLMNOPQRSTUVWXYZ", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see test string with saturation into 1st row.\n");

  fprintf(stdout,"  Press <ENTER> for cursor in left, upper corner (HOME).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_setcursorpos(p_display, 0, 0);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: set cursor FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see the cursor in the left, upper corner;\n");
  fprintf(stdout,"            On the capital 'I'.\n");

  fprintf(stdout,"  Press <ENTER> for scroll by one row.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_scroll(p_display, 1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: scroll FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see a one-row scroll.\n");

  fprintf(stdout,"  Press <ENTER> for scroll by two rows.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_scroll(p_display, 2);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: scroll FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see a two-row scroll (3 rows total).\n");

  fprintf(stdout,"  Press <ENTER> to clear display.\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_clear(p_display);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: clear display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] clear display.\n");

  fprintf(stdout,"  Press <ENTER> for multi-line printing (autoscroll ACTIVE!).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_autoscroll(p_display, true);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: set autoscroll FAILED.\n");
    goto DoError;
  }

  retval = ch341a_hd44780u_print(p_display, "This is a three-row message automatically wrapped.\n", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see the three-rows-string (cursor in fourth row).\n");

  fprintf(stdout,"  Press <ENTER> for just 'Test #4' printing (no newline).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "Test #4", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see the 'Test #4' with cursor right behind.\n");

  fprintf(stdout,"  Press <ENTER> for just a NEWLINE printing (autoscroll ACTIVE!).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "\n", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see the 'Test #4' no in row 3 and cursor in row 4.\n");

  fprintf(stdout,"  Press <ENTER> for very-long printing (autoscroll ACTIVE!).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_print(p_display, "This is a really long message, which does not fit onto the display.\nThe library automatically scrolls the display.\nYou should see only this trailing message.", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }
  fprintf(stdout,"  [CHECK!] You should see on the display now:\n");
  fprintf(stdout,"  1: 'splay.              '\n");
  fprintf(stdout,"  2: 'You should see only '\n");
  fprintf(stdout,"  3: 'this trailing messag'\n");
  fprintf(stdout,"  4: 'e.                  '\n\n");

  fprintf(stdout,"  Press <ENTER> for a real live simulation:\n");
  fprintf(stdout,"  A 'virtual' Linux is booted with systemd.\n");
  fprintf(stdout,"  After a short delay, the current status\n");
  fprintf(stdout,"  of a 'virtual' host is displayed for\n");
  fprintf(stdout,"  several seconds. After that, the test stops\n");
  fprintf(stdout,"  clearing the screen (cursor is off).\n");
  fgets(buffer, sizeof(buffer), stdin);

  retval = ch341a_hd44780u_clear(p_display);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: clear display.\n");
    goto DoError;
  }

  retval = ch341a_hd44780u_print(p_display, "Booting linux...", sizeof("Booting linux...")-1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print message.\n");
    goto DoError;
  }
  ms_sleep(2000);

  retval = ch341a_hd44780u_print(p_display, "\nChecking filesystem(s)...", sizeof("\nChecking filesystem(s)...")-1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print message.\n");
    goto DoError;
  }
  ms_sleep(2000);

  retval = ch341a_hd44780u_print(p_display, "DONE!", sizeof("DONE!")-1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print message.\n");
    goto DoError;
  }
  ms_sleep(2000);

  retval = ch341a_hd44780u_print(p_display, "\nsystemd reached multi-user target.", sizeof("\nsystemd reached multi-user target.")-1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print message.\n");
    goto DoError;
  }
  ms_sleep(2000);

  retval = ch341a_hd44780u_configure(p_display, true, false, false);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: switch cursor OFF.\n");
    goto DoError;
  }

  retval = ch341a_hd44780u_clear(p_display);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: clear display\n");
    goto DoError;
  }

  retval = ch341a_hd44780u_autoscroll(p_display, false);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: disable autoscroll\n");
    goto DoError;
  }

  retval = ch341a_hd44780u_print(p_display, "load: 100.00%       temp: 100.0 C       mem: 10.5G/64.0G av.10000 process(es)   ", -1);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: printing\n");
    goto DoError;
  }

  srand((unsigned int)time(NULL));

  for (i=0;i<10;i++)
  {
    uint32_t load = ((uint32_t)rand()) % 10001;
    uint32_t temp = ((uint32_t)rand()) % 1001;
    uint32_t mem = ((uint32_t)rand()) % 640;
    uint32_t processes = (((uint32_t)rand()) % 10000) + 1/*this is init :-)*/;

    snprintf(buffer,sizeof(buffer), "%3u.%02u", load / 100, load % 100 );
    retval = ch341a_hd44780u_setcursorpos(p_display, 6, 0);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: set cursor pos.\n");
      goto DoError;
    }
    retval = ch341a_hd44780u_print(p_display, buffer, -1);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: printing\n");
      goto DoError;
    }

    snprintf(buffer,sizeof(buffer), "%3u.%u", temp / 10, temp % 10 );
    retval = ch341a_hd44780u_setcursorpos(p_display, 6, 1);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: set cursor pos.\n");
      goto DoError;
    }
    retval = ch341a_hd44780u_print(p_display, buffer, -1);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: printing\n");
      goto DoError;
    }

    snprintf(buffer,sizeof(buffer), "%2u.%u", mem / 10, mem % 10 );
    retval = ch341a_hd44780u_setcursorpos(p_display, 5, 2);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: set cursor pos.\n");
      goto DoError;
    }
    retval = ch341a_hd44780u_print(p_display, buffer, -1);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: printing\n");
      goto DoError;
    }

    snprintf(buffer,sizeof(buffer), "%5u", processes );
    retval = ch341a_hd44780u_setcursorpos(p_display, 0, 3);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: set cursor pos.\n");
      goto DoError;
    }
    retval = ch341a_hd44780u_print(p_display, buffer, -1);
    if (0 != retval)
    {
      fprintf(stderr,"ERROR: printing\n");
      goto DoError;
    }

    ms_sleep(1000);
  }

  fprintf(stdout,"\nTEST complete, programming German umlauts.\n");

  retval = ch341a_hd44780u_program_german_umlauts ( p_display );
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: program German umlauts\n");
    goto DoError;
  }

  retval = ch341a_hd44780u_clear(p_display);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: clear screen\n");
    goto DoError;
  }

  // Ä = 0, Ö = 1, Ü = 2, ä = 3, ö = 4, ü = 5, ß = 6

  retval = ch341a_hd44780u_print(p_display, "\000pfel \005berall\n\000 \001 \002 \003 \004 \005\nV\004gel fliegen\nStra\006e (street)", 55);
  if (0 != retval)
  {
    fprintf(stderr,"ERROR: print on display FAILED.\n");
    goto DoError;
  }

  fprintf(stdout,"  You should see now four lines (English translations):\n");
  fprintf(stdout,"  1: 'apples everywhere'\n");
  fprintf(stdout,"  2: printout of six umlauts\n");
  fprintf(stdout,"  3: 'birds fly'\n");
  fprintf(stdout,"  4: 'street' followed by '(street)'\n\n");

  fprintf(stdout,"DONE!\n");

  return 0;
}

#define IS_HEX_DIGIT(_c)      ( (((_c)>='0') && ((_c)<='9')) | (((_c)>='A') && ((_c)<='F')) | (((_c)>='a') && ((_c)<='f')) )

int main ( int argc, char *argv[] )
{
  int                 i, retval, rc = 0;
  bool                have_test = false;
  uint32_t            device = 0, new_device;
  p_ch341a_hd44780u   p_display = NULL;
  const char         *p;
  char               *endptr;
  uint32_t            cur_x, cur_y;
  bool                display_on, cursor_on, blinking_on, autoscroll_on, double_quote;
  uint8_t             buffer[256];

  if (1 == argc)
  {
ShowHelp:
    fprintf(stdout,"HD44780U LCD display cmdline tool\n\n");
    fprintf(stdout,"usage: %s [<options>...]\n",argv[0]);
    fprintf(stdout,"------\n\n");
    fprintf(stdout,"  --help            (or no options at all): print this help.\n");
    fprintf(stdout,"The following options may be repeated:\n");
    fprintf(stdout,"  --dev=<num>       specify device (default: 0)\n");
    fprintf(stdout,"  --clear           clear display\n");
    fprintf(stdout,"  --conf=DCBA       leave empty or specifc one or more of 'D', 'C', 'B':\n");
    fprintf(stdout,"                    'D' display on (if omitted, then off)\n");
    fprintf(stdout,"                    'C' cursor on (if omitted, then off)\n");
    fprintf(stdout,"                    'B' cursor blinking on (if omitted, then off)\n");
    fprintf(stdout,"                    'A' autoscroll on (if omitted, then off)\n");
    fprintf(stdout,"  --cursor=x,y      set cursor position; x=0..19, y=0..3\n");
    fprintf(stdout,"  --scroll=rows     rows=1..4, scroll this many rows\n");
    fprintf(stdout,"  --print=\"<msg>\" print this message, '\\n' is newline\n");
    fprintf(stdout,"                    '\\\"' is double quote, '\\xNN' is code (hex).\n");
    fprintf(stdout,"                    '\\\'' is prime; you can alternatively use:\n");
    fprintf(stdout,"                    --print='<msg>'\n");
    fprintf(stdout,"  --wait=millis     delay this amount of milliseconds.\n");
    fprintf(stdout,"  --test            run INTERACTIVE test suite (all other options\n");
    fprintf(stdout,"                    except for --dev are IGNORED.\n\n");
    if (NULL != p_display)
      ch341a_hd44780u_close(p_display);
    return 1;
  }

  for (i=1;i<argc;i++)
  {
    if (!memcmp(argv[i],"--help",sizeof("--help")-1))
      goto ShowHelp;
    if (!memcmp(argv[i],"--test",sizeof("--test")-1))
    {
      have_test = true;
      break;
    }
  }

  if (have_test)
  {
    for (i = 1; i < argc; i++)
    {
      if (!memcmp(argv[i],"--dev=",sizeof("--dev=")-1)) // only recognize first --dev argument
      {
        device = (uint32_t)strtoul(argv[i]+sizeof("--dev=")-1, NULL, 10);
        break;
      }
    }

    p_display = ch341a_hd44780u_open(device);
    if (NULL == p_display)
    {
      fprintf(stderr,"ERROR: unable to open device with zero-based index %u\n",device);
      fprintf(stderr,"------ errno=%i: %s\n",errno,strerror(errno));
      return 1;
    }

    retval = run_testsuite(p_display, device);

    ch341a_hd44780u_close(p_display);

    return retval;
  } // of have_test

  //////////////////////////////////////////////////////////////

  for (i = 1; i < argc; i++)
  {
    if (!memcmp(argv[i],"--dev=",sizeof("--dev=")-1))
    {
      new_device = (uint32_t)strtoul(argv[i]+sizeof("--dev=")-1, NULL, 10);
      if (!(new_device == device && NULL != p_display))
      {
        if (NULL != p_display)
          ch341a_hd44780u_close(p_display);
        device = new_device;
        p_display = ch341a_hd44780u_open(device);
        if (NULL == p_display)
        {
          fprintf(stderr,"ERROR: unable to open device with zero-based index %u\n",device);
PrintErrno:
          if (NULL != p_display)
            ch341a_hd44780u_close(p_display);
          fprintf(stderr,"------ errno=%i: %s\n",errno,strerror(errno));
          return 1;
        }
      }
    }
    else
    if (!strcmp(argv[i],"--clear"))
    {
      if (NULL == p_display)
      {
        p_display = ch341a_hd44780u_open(device);
        if (NULL == p_display)
        {
          fprintf(stderr,"ERROR: unable to open device with zero-based index %u\n",device);
          goto PrintErrno;
        }
      }

      if (0 != ch341a_hd44780u_clear(p_display))
      {
        fprintf(stderr,"ERROR: unable to clear display.\n");
        goto PrintErrno;
      }
    }
    else
    if (!memcmp(argv[i],"--conf=",sizeof("--conf=")-1))
    {
      p = argv[i]+sizeof("--conf=")-1;
      display_on = cursor_on = blinking_on = autoscroll_on = false;
      while (0 != *p)
      {
        switch(*p)
        {
          case 'D': display_on = true; break;
          case 'C': cursor_on = true; break;
          case 'B': blinking_on = true; break;
          case 'A': autoscroll_on = true; break;
          default:
            goto ShowHelp;
        }
        p++;
      }

      if (NULL == p_display)
      {
        p_display = ch341a_hd44780u_open(device);
        if (NULL == p_display)
        {
          fprintf(stderr,"ERROR: unable to open device with zero-based index %u\n",device);
          goto PrintErrno;
        }
      }

      if (0 != ch341a_hd44780u_configure(p_display, display_on, cursor_on, blinking_on))
      {
        fprintf(stderr,"ERROR: unable to configure display (on/off, cursor, blinking).\n");
        goto PrintErrno;
      }

      if (0 != ch341a_hd44780u_autoscroll(p_display, autoscroll_on))
      {
        fprintf(stderr,"ERROR: unable to configure auto-scroll on/off.\n");
        goto PrintErrno;
      }
    }
    else
    if (!memcmp(argv[i],"--cursor=",sizeof("--cursor=")-1))
    {
      p = argv[i] + sizeof("--cursor=") - 1;
      cur_x = strtoul(p, &endptr, 10);
      if (NULL == endptr || ',' != *endptr)
        goto ShowHelp;
      cur_y = strtoul(endptr+1, NULL, 10);

      if (NULL == p_display)
      {
        p_display = ch341a_hd44780u_open(device);
        if (NULL == p_display)
        {
          fprintf(stderr,"ERROR: unable to open device with zero-based index %u\n",device);
          goto PrintErrno;
        }
      }

      if (0 != ch341a_hd44780u_setcursorpos(p_display, cur_x, cur_y))
      {
        fprintf(stderr,"ERROR: unable to set cursor position to %u,%u\n",cur_x, cur_y);
        goto PrintErrno;
      }
    }
    else
    if (!memcmp(argv[i],"--wait=",sizeof("--wait=")-1))
    {
      p = argv[i] + sizeof("--wait=") - 1;
      cur_x = strtoul(p, NULL, 10);
      ms_sleep(cur_x);
    }
    else
    if (!memcmp(argv[i],"--print=",sizeof("--print=")-1))
    {
      p = argv[i] + sizeof("--print=") - 1;
      cur_x = 0;
      memset(buffer,0,sizeof(buffer));
      switch(*p)
      {
        case '\'':
          double_quote = false;
          p++;
          break;
        default:
          double_quote = true;
          break;
      }

      while (0 != *p)
      {
        switch(*p)
        {
          case 0:
            goto StringComplete;
          case '"':
            if (!double_quote)
              goto ShowHelp;
            goto StringComplete;
          case '\'':
            if (double_quote)
              goto ShowHelp;
            goto StringComplete;
          case '\\':
            p++;
            switch(*p)
            {
              case '"':
              case '\'':
              case '\\':
              case 'n':
                if (cur_x == sizeof(buffer))
                {
                  fprintf(stderr,"ERROR: buffer exhausted.\n");
                  rc = 1;
                  goto Exit;
                }
                buffer[cur_x++] = ('n' == *p) ? 0x0A : ((uint8_t)*p);
                p++;
                break;
              case 'x':
                if ((!IS_HEX_DIGIT(p[1]) || !IS_HEX_DIGIT(p[2])))
                  goto ShowHelp;
                if (cur_x == sizeof(buffer))
                {
                  fprintf(stderr,"ERROR: buffer exhausted.\n");
                  rc = 1;
                  goto Exit;
                }
                cur_y = ((uint8_t)p[1])-0x30;
                if (cur_y > 9) cur_y -= 7;
                if (cur_y > 9) cur_y -= 32;
                buffer[cur_x] = (uint8_t)(cur_y << 4);

                cur_y = ((uint8_t)p[2])-0x30;
                if (cur_y > 9) cur_y -= 7;
                if (cur_y > 9) cur_y -= 32;
                buffer[cur_x++] |= (uint8_t)cur_y;
                p+=3;
                break;
              case '0':
              case '1':
              case '2':
              case '3':
                if (p[1]<'0' || p[1]>'7' || p[2]<'0' || p[2]>'7')
                  goto ShowHelp;
                if (cur_x == sizeof(buffer))
                {
                  fprintf(stderr,"ERROR: buffer exhausted.\n");
                  rc = 1;
                  goto Exit;
                }
                buffer[cur_x++] = (((uint8_t)(p[0] - 0x30)) << 6) | (((uint8_t)(p[1] - 0x30)) << 3) | ((uint8_t)(p[2] - 0x30));
                p += 3;
                break;

              default:
                goto ShowHelp;
            }

            break;

          default:
            if (cur_x == sizeof(buffer))
            {
              fprintf(stderr,"ERROR: buffer exhausted.\n");
              rc = 1;
              goto Exit;
            }
            buffer[cur_x++] = *(p++);
            break;
        }
      } // of string parsing

StringComplete:

      if (NULL == p_display)
      {
        p_display = ch341a_hd44780u_open(device);
        if (NULL == p_display)
        {
          fprintf(stderr,"ERROR: unable to open device with zero-based index %u\n",device);
          goto PrintErrno;
        }
      }

      if (0 != ch341a_hd44780u_print(p_display, (const char *)buffer, cur_x))
      {
        fprintf(stderr,"ERROR: unable to send string to the display.\n");
        goto PrintErrno;
      }
    }
    else
      goto ShowHelp;
  } // of for all arguments

Exit:

  if (NULL != p_display)
    ch341a_hd44780u_close(p_display);

  return rc;
}


#if 0
fprintf(stdout,"  --scroll=rows     rows=1..4, scroll this many rows\n");
fprintf(stdout,"  --print=\"<msg>\" print this message, '\\n' is newline\n");
fprintf(stdout,"                    '\\\"' is double quote, '\\xNN' is code (hex).\n");
fprintf(stdout,"  --wait=millis     delay this amount of milliseconds.\n");
#endif




