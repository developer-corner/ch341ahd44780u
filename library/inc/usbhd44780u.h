#ifndef _INC_USBHD44780U_H_
#define _INC_USBHD44780U_H_

/**
 * @brief   userland C library for accessing an HD44780U LCD display
 *          (4 rows, 20 columns) connected to the host via the
 *          USB Bridge Controller CH341A (VID:PID 1A86:5512)
 * @author  Ingo A. Kubbilun (ingo.kubbilun@gmail.com)
 *
 * You have to load the Linux kernel driver ch341ahd44780u.ko if
 * it is not automatically loaded.
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>

typedef struct _ch341a_hd44780u         ch341a_hd44780u, *p_ch341a_hd44780u;

/**
 * @brief opens the LCD display HD44780u connected via the USB Bridge Controller CH341A
 *        (the Linux kernel driver ch341ahd44780u.ko has to be loaded first!)
 *
 * @param[in]     device_number       zero-based index of the display to be opened
 *                                    (you may attach multiple displays to the host)
 *
 * @return NULL on error (see errno) or the pointer to a new internal structure
 *         managing an HD44780u display connected via CH341A (USB).
 */
p_ch341a_hd44780u ch341a_hd44780u_open ( uint32_t device_number );

/**
 * @brief closes the LCD display and frees all resources associated with it
 *
 * @param[in]     pointer to device
 */
void ch341a_hd44780u_close ( p_ch341a_hd44780u p_display );

/**
 * @brief configures the display: on/off, cursor on/off, cursor blinking on/off
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     on                  true to switch LCD display on
 * @param[in]     cursor              true to show cursor
 * @param[in]     blinking            true to let cursor blink
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_configure ( p_ch341a_hd44780u p_display, bool on, bool cursor, bool blinking );

/**
 * @brief clears the display; if the cursor is on, it is positioned in the upper, left corner
 *
 * @param[in]     p_display           pointer to the opened display
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_clear ( p_ch341a_hd44780u p_display );

/**
 * @brief sets the cursor position (or next write position if cursor not shown)
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     x                   x (column) 0..19
 * @param[in]     y                   y (row) 0..3
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_setcursorpos ( p_ch341a_hd44780u p_display, uint32_t x, uint32_t y );

/**
 * @brief sets the cursor position (or next write position if cursor not shown)
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     p_x                 receives x (column) 0..19
 * @param[in]     p_y                 receives y (row) 0..3
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_getcursorpos ( p_ch341a_hd44780u p_display, uint32_t *p_x, uint32_t *p_y );

/**
 * @brief either disables or enables auto-scroll mode: if enabled, the display
 *        automatically scrolls down if the last position (19,3) was written by
 *        one row (line).
 *        If disabled, the next write (cursor) position becomes HOME (0,0).
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     autoscroll          true to enable auto-scroll
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_autoscroll ( p_ch341a_hd44780u p_display, bool autoscroll );

/**
 * @brief scrolls the LCD display by a number of rows
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     rows                number of rows (1..4); if 0 specified
 *                                    this is a no-op; if > 4 specified, this becomes
 *                                    4 (meaning empty display).
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_scroll ( p_ch341a_hd44780u p_display, uint32_t rows );

/**
 * @brief prints one or more characters on the LCD display advancing the current
 *        cursor position
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     p_string            pointed to possibly zero-terminated string
 * @param[in]     l_string            -1 if string is zero-terminated or the number
 *                                    of characters to be printed otherwise. You
 *                                    can print the character zero only if you specify
 *                                    a meaningful value here.
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_print ( p_ch341a_hd44780u p_display, const char *p_string, int32_t l_string );

/**
 * @brief retrieves the backed up (in host RAM!) display buffer
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in/out] p_displaybuffer     pointer to 4x20 = 80 bytes (IN); filled with
 *                                    currently displayed content (OUT)
 *
 * @return -1 on error (parameter error) or 0 on success.
 */
int ch341a_hd44780u_get_displaybuffer ( p_ch341a_hd44780u p_display, uint8_t *p_displaybuffer );

/**
 * @brief The HD44780U can be programmed with up to eight (8) user-defined characters.
 *
 *        This function programs the CGRAM (character generator RAM) for one of the eight
 *        (0..7) characters in 5x8 format. The 8th row should be all zero bits because this
 *        is the cursor row. The bits 7,6,5 are zeroed-out by this function of one dot matrix
 *        row. The bits 4,3,2,1,0 represent the pixels (1=on, 0=off) of one character row.
 *
 * @param[in]     p_display           pointer to the opened display
 * @param[in]     chidx               user-defined character index 0..7
 * @param[in]     p_dotmatrix         pointer to eight bytes with the pixels (one bit per
 *                                    pixel) of a 5x8 character. The three most significant
 *                                    bits are automatically zeroed out by this function.
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_program_cgram ( p_ch341a_hd44780u p_display, uint32_t chidx, const uint8_t *p_dotmatrix );

/**
 * @brief programs the German umlauts Ä Ö Ü ä ö ü ß into CGRAM as characters 0..6
 *
 * @param[in]     p_display           pointer to the opened display
 *
 * @return 0 (OK) or -1 on error (see errno)
 */
int ch341a_hd44780u_program_german_umlauts ( p_ch341a_hd44780u p_display );

#ifdef __cplusplus
}
#endif

#endif /* _INC_USBHD44780U_H_ */
