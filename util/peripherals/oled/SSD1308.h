#include <stdint.h>
#include <I2C.h>
#include "platform/Stream.h"
/** @file SSD1308 I2C device class header file
 *   Based on Solomon Systech SSD1308 datasheet, rev. 1, 10/2008
 *   The SSD1308 is used for example in the Seeed 128x64 OLED Display
 *   http://www.seeedstudio.com/depot/grove-oled-display-12864-p-781.html?cPath=163_167
 */
// The original code by Andrew Schamp is using (and has been submitted as a part of) Jeff Rowberg's I2Cdevlib library,
// which should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// Some parts also mashed up from Graphic Library for driving monochrome displays based on the PCD8544,
// Copyright (c) 2011, Wim De Roeve, who in turn did partial port of code found on
// http://serdisplib.sourceforge.net/ser/pcd8544.html#links and by Petras Saduikis <petras@petras.co.uk>
//
// Changelog:
//   2011-08-25 - Initial release by Andrew Schamp <schamp@gmail.com>
//   2012-06-19 - Ported to mbed and optimised (WH)
//   2013-07-12 - Minor comment fix and placeholder for SSD1306 (WH)
//   2015-01-01 - Switch for optimised I2C calls to test on F401 (WH)
//   2017-12-18 - Fixed non-copyable issue (Thx kenjiArai)
//             
/* 
================================================================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Andrew Schamp
Copyright (c) 2012,2013,2017 WH (mbed port)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
================================================================================
*/

#ifndef SSD1308_H
#define SSD1308_H

// This is the I2C address (8 bit)
//  There are two possible addresses: with D/C# (pin 13) grounded, the address is 0x78,
//  with D/C# tied high it is 0x7A. Assume grounded by default.
#define SSD1308_SA0                0x78
#define SSD1308_SA1                0x7A
#define SSD1308_DEF_SA             SSD1308_SA0

// Display dimensions
#define ROWS                       64
#define COLUMNS                    128
#define PAGES                      (ROWS / 8)
#define MAX_PAGE                   (PAGES - 1)
#define MAX_ROW                    (ROWS - 1)
#define MAX_COL                    (COLUMNS - 1)

// Character dimensions 8x8 font
#define CHARS                      (COLUMNS / FONT8x8_WIDTH)

// Command and Datamode 
#define COMMAND_MODE               0x80 // continuation bit is set!
#define DATA_MODE                  0x40

// Commands and Parameter defines
#define SET_LOWER_COLUMN           0x00 // | with lower nibble  (Page mode only)
#define SET_HIGHER_COLUMN          0x10 // | with higher nibble (Page mode only)

#define HORIZONTAL_ADDRESSING_MODE 0x00
#define VERTICAL_ADDRESSING_MODE   0x01
#define PAGE_ADDRESSING_MODE       0x02
#define SET_MEMORY_ADDRESSING_MODE 0x20 // takes one byte as given above

#define SET_COLUMN_ADDRESS         0x21 // takes two bytes, start address and end address of display data RAM
#define SET_PAGE_ADDRESS           0x22 // takes two bytes, start address and end address of display data RAM

// Command maybe unsupported by SSD1308
#define FADE_INTERVAL_8_FRAMES     0x00
#define FADE_INTERVAL_16_FRAMES    0x01
#define FADE_INTERVAL_24_FRAMES    0x02
#define FADE_INTERVAL_32_FRAMES    0x03
#define FADE_INTERVAL_64_FRAMES    0x07
#define FADE_INTERVAL_128_FRAMES   0x0F
#define FADE_BLINK_DISABLE         0x00
#define FADE_OUT_ENABLE            0x20
#define BLINK_ENABLE               0x30
#define SET_FADE_BLINK             0x23 // takes one byte
                                        //  bit5-4 = 0, fade/blink mode
                                        //  bit3-0 = Time interval in frames 

#define SET_DISPLAY_START_LINE     0x40 // | with a row number 0-63 to set start row. (Reset = 0)

#define SET_CONTRAST               0x81 // takes one byte, 0x00 - 0xFF

#define SET_SEGMENT_REMAP_0        0xA0 // column address 0 is mapped to SEG0 (Reset)
#define SET_SEGMENT_REMAP_127      0xA1 // column address 127 is mapped to SEG0

#define SET_DISPLAY_GDDRAM         0xA4 // restores display to contents of RAM
#define SET_ENTIRE_DISPLAY_ON      0xA5 // turns all pixels on, does not affect RAM

#define SET_NORMAL_DISPLAY         0xA6 // a databit of 1 indicates pixel 'ON'
#define SET_INVERSE_DISPLAY        0xA7 // a databit of 1 indicates pixel 'OFF'

#define SET_MULTIPLEX_RATIO        0xA8 // takes one byte, from 16xMUX to 64xMUX (MUX Ratio = byte+1; Default 64)

#define EXTERNAL_IREF              0x10
#define INTERNAL_IREF              0x00
#define SET_IREF_SELECTION         0xAD // sets internal or external Iref

#define SET_DISPLAY_POWER_OFF      0xAE
#define SET_DISPLAY_POWER_ON       0xAF

#define PAGE0                      0x00
#define PAGE1                      0x01
#define PAGE2                      0x02
#define PAGE3                      0x03
#define PAGE4                      0x04
#define PAGE5                      0x05
#define PAGE6                      0x06
#define PAGE7                      0x07
#define SET_PAGE_START_ADDRESS     0xB0 // | with a page number to get start address (Page mode only)

#define SET_COMMON_REMAP_0         0xC0 // row address  0 is mapped to COM0 (Reset)
#define SET_COMMON_REMAP_63        0xC8 // row address 63 is mapped to COM0

#define SET_DISPLAY_OFFSET         0xD3 // takes one byte from 0-63 for vertical shift, Reset = 0

#define SET_DISPLAY_CLOCK          0xD5 // takes one byte
                                        //  bit7-4 = Osc Freq DCLK (Reset = 1000b) 
                                        //  bit3-0 = Divide ration (Reset = oooob, Ratio = 1)   

#define SET_PRECHARGE_TIME         0xD9 // takes one byte
                                        //  bit7-4 = Phase2, upto 15 DCLKs (Reset = 0010b) 
                                        //  bit3-0 = Phase1, upto 15 DCLKs (Reset = 0010b)   

                                       
#define COMMON_BASE                0x02 // 
#define COMMON_SEQUENTIAL          0x00 // Sequential common pins config
#define COMMON_ALTERNATIVE         0x10 // Odd/Even common pins config (Reset)
#define COMMON_LEFTRIGHT_NORMAL    0x00 // LeftRight Normal (Reset)
#define COMMON_LEFTRIGHT_FLIP      0x20 // LeftRight Flip 
#define SET_COMMON_CONF            0xDA // takes one byte as given above


#define VCOMH_DESELECT_0_65_CODE   0x00
#define VCOMH_DESELECT_0_77_CODE   0x20
#define VCOMH_DESELECT_0_83_CODE   0x30
#define SET_VCOMH_DESELECT_LEVEL   0xDB // takes one byte as given above

#define NOP                        0xE3

#define SCROLL_INTERVAL_5_FRAMES   0x00
#define SCROLL_INTERVAL_64_FRAMES  0x01
#define SCROLL_INTERVAL_128_FRAMES 0x02
#define SCROLL_INTERVAL_256_FRAMES 0x03
#define SCROLL_INTERVAL_3_FRAMES   0x04
#define SCROLL_INTERVAL_4_FRAMES   0x05
#define SCROLL_INTERVAL_25_FRAMES  0x06
#define SCROLL_INTERVAL_2_FRAMES   0x07

#define SET_RIGHT_HOR_SCROLL       0x26 // takes 6 bytes: 0x00, PageStart, Scroll_Interval, PageEnd, 0x00, 0xFF
#define SET_LEFT_HOR_SCROLL        0x27 // takes 6 bytes: 0x00, PageStart, Scroll_Interval, PageEnd, 0x00, 0xFF

#define SET_VERT_RIGHT_HOR_SCROLL  0x29 // takes 5 bytes: 0x00, PageStart, Scroll_Interval, PageEnd, VertOffset
#define SET_VERT_LEFT_HOR_SCROLL   0x2A // takes 5 bytes: 0x00, PageStart, Scroll_Interval, PageEnd, VertOffset

#define SET_DEACTIVATE_SCROLL      0x2E
#define SET_ACTIVATE_SCROLL        0x2F

#define SET_VERTICAL_SCROLL_AREA   0xA3 // takes 2 bytes: Rows in Top Area (Reset=0), Rows in Scroll Area (Reset=64)



/** Class to control an SSD1308 based oled Display
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include "mbed_logo.h"
 * #include "SSD1308.h"
 
 * //Pin Defines for I2C Bus
 * #define D_SDA                  p28
 * #define D_SCL                  p27
 * I2C i2c(D_SDA, D_SCL);
 *
 * // Host PC Communication channels
 * Serial pc(USBTX, USBRX); // tx, rx
 *
 * // Instantiate OLED
 * SSD1308 oled = SSD1308(&i2c, SSD1308_SA0);
 * 
 * int main() {
 *   pc.printf("OLED test start\r");  
 *   oled.writeString(0, 0, "Hello World !");
 *   //  oled.printf("Hello World !");
 * 
 *   oled.fillDisplay(0xAA);
 *   oled.setDisplayOff();
 *   wait(1);   
 *   oled.setDisplayOn();
 * 
 *   oled.clearDisplay();
 *   oled.setDisplayInverse();
 *   wait(0.5);
 *   oled.setDisplayNormal();                                         
 *
 *   oled.writeBitmap((uint8_t*) mbed_logo);
 *
 *   pc.printf("OLED test done\r\n");  
 * }
 *
 * @endcode
 */
class SSD1308 : public mbed::Stream {
  public:
    
/**
  *@brief Constructor
  *@param I2C &i2c reference to i2c,
  *@param uint8_t deviceAddress slaveaddress (8bit to use for the controller (0x78 by default, assumes D/C# (pin 13) grounded)
 */    
    SSD1308(mbed::I2C *i2c, uint8_t address = SSD1308_DEF_SA);

// High Level methods

/** @brief High level Init, most settings remain at Power-On reset value
 */
    void initialize();

/** @brief clear the display
*/
    void clearDisplay();


/** @brief fill the display
 *  @param uint8_t pattern fillpattern vertical patch or 8 bits 
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t start_col  begin column (0..MAX_COL)
 *  @param uint8_t end_col    end column   (start_col..MAX_COL)
*/ 
    void fillDisplay(uint8_t pattern = 0x00,
                     uint8_t start_page=0, uint8_t end_page=MAX_PAGE,
                     uint8_t start_col=0, uint8_t end_col=MAX_COL);


/** @brief write a bitmap to the display
 *  @param uint8_t* data pointer to bitmap
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t start_col  begin column (0..MAX_COL)
 *  @param uint8_t end_col    end column   (start_col..MAX_COL)
*/  
    void writeBitmap(uint8_t* data,
                     uint8_t start_page=0, uint8_t end_page=MAX_PAGE,
                     uint8_t start_col=0, uint8_t end_col=MAX_COL);
    
/** @brief write a level meter to the display, Width is (PRG_MAX_SCALE + 2) pixels
 *  @param uint8_t page begin page   (0..MAX_PAGE)
 *  @param uint8_t col  begin column (0..MAX_COL)
 *  @param int percentage value      (0..100)
*/
    void writeProgressBar(uint8_t page, uint8_t col, int percentage);


/** @brief write a level meter to the display, Width is (PRG_MAX_SCALE + 2) pixels
 *  @param uint8_t page begin page   (0..MAX_PAGE)
 *  @param uint8_t col  begin column (0..MAX_COL)
 *  @param int percentage value      (0..100)
*/
    void writeLevelBar(uint8_t page, uint8_t col, int percentage);

    //void setXY(uint8_t, uint8_t y);
    
    // Select inverted or normal text    
    void setInverted(bool inverted) { _inverted = inverted; };    
        
/** @brief Write single character to the display using the 8x8 fontable
 *  @brief Start at current cursor location
 *  @param char chr character to write
*/ 
    void writeChar(char chr);  
    
/** @brief Write large character (16x24 font)
 *  @param uint8_t row  row number    (0...MAX_ROW)
 *  @param uint8_t col  column number (0...MAX_COL)
 *  @param char chr     Used for displaying numbers 0 - 9 and '+', '-', '.'
 */   
    void writeBigChar(uint8_t row, uint8_t col, char chr);      
    
/** @brief Write a string to the display using the 8x8 font
 *  @brief Start at selected cursor location, text will wrap around until it is done
 *  @param uint8_t row  row number    (0...ROWS/FONT_HEIGHT)
 *  @param uint8_t col  column number (0...COLUMNS/FONT_WIDTH)
 *  @param const char * text pointer to text
 */   
    void writeString(uint8_t row, uint8_t col, const char* txt);
    
    // Stream implementation - provides printf() interface
    // You would otherwise be forced to use writeChar() or writeString()
    virtual int _putc(int value) { writeChar(value); return 1; };
    virtual int _getc() { return -1; };
   
// Future extension with graphics features
    // this must be defined by the subclass
//    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;
    
// Medium Level methods

/** @brief Set Horizontal Addressing Mode (cursor incr left-to-right, top-to-bottom)
 * 
 */
    void setHorizontalAddressingMode();
    
/** @brief Set Vertical Addressing Mode  (cursor incr top-to-bottom, left-to-right)
 * 
 */
    void setVerticalAddressingMode();
    
/** @brief Set Page Addressing Mode  (cursor incr left-to-right)
 * 
 */
    void setPageAddressingMode();
        
/** @brief Set Addressing Mode
 *  @param uint8_t mode 
 */
    void setMemoryAddressingMode(uint8_t mode);


/** 
 *  @brief Set Column Start (for Page Addressing Mode only)
 *  @param uint8_t column column start (valid range 0..MAX_COLS)
 */    
    void setColumnStartForPageAddressingMode(uint8_t column);
    
/** 
 *  @brief Set Page Start (for Page Addressing Mode only)
 *  @param uint8_t page page start (valid range PAGE0 - PAGE7)
 */    
    void setPageStartForPageAddressingMode(uint8_t page);
   


/** @param uint8_t start startcolumn (valid range 0..MAX_COL)
 *  @param uint8_t end   endcolumn   (valid range start..MAX_COL)
 */   
    void setColumnAddress(uint8_t start, uint8_t end);
    
/** @param uint8_t start startpage (valid range 0..MAX_PAGE)
 *  @param uint8_t end   endpage   (valid range start..MAX_PAGE)
 */  
    void setPageAddress(uint8_t start, uint8_t end);
    

/** 
 *  @brief Set Display StartLine, takes one byte, 0x00-0x3F
 *  @param uint8_t line startline (valid range 0..MAX_ROWS)
 */  
    void setDisplayStartLine(uint8_t line);
    
/** @brief Set Contrast
 *  @param uint8_t contrast (valid range 0x00 (lowest) - 0xFF (highest))
*/    
    void setContrastControl(uint8_t contrast);


/** @brief Shows All Pixels On
 */
    void setEntireDisplayOn();
    
/** @brief Shows Pixels as RAM content
 */
    void setEntireDisplayRAM();
    
/** @brief Shows Pixels On or as RAM content
 *  @param bool on (true is All on, false is RAM content)
 */
    void setEntireDisplay(bool on);
    
 
    // @brief Set Display line MPX Ratio, takes one byte, 0x00-0x3F
    // @param uint8_t lines (valid range 0..MAX_ROWS)
    void setMultiplexRatio(uint8_t lines);


/** @brief Sets Internal Iref
 */
    void setInternalIref();

/** @brief Sets External Iref (default)
 */   
    void setExternalIref();


/** @brief Enable Display
*/    
    void setDisplayOn();

/** @brief Disable Display
*/     
    void setDisplayOff();

/** @brief Enable or Disable Display
 *  @param bool on
 */    
    void setDisplayPower(bool on);

/** @brief Show White pixels on Black background
 */ 
    void setDisplayNormal();

/** @brief Show Black pixels on White background
 */   
    void setDisplayInverse();

/** @brief Blink display by fading in and out over a set number of frames
 *  @param bool on
 */
    void setDisplayBlink(bool on);  
    
/** @brief Fade out display in set number of frames
 *  @param bool on
 */        
    void setDisplayFade(bool on);    
    
/** @brief Display Flip (Left/Right, Up/Down)
 *  @param bool left flip Left/Right
 *  @param bool down flip Up/Down
 */  
    void setDisplayFlip(bool left, bool down);

    // Set vertical shift by COM from 0 - 63 (0x00 - 0x3F) (Reset = 0x00)
    void setDisplayOffset(uint8_t offset);
    
    // Oscillator freq 0x00-0x0F (reset 0x08)
    // Divide ratio 0x00-0x0F, value +1 (reset 0x00)
    void setDisplayClock(uint8_t divideRatio, uint8_t oscFreq);
    
    // Phase1 0x01-0x0F period of up to 15 DCLK clocks (reset 0x02, 0 is invalid)
    // Phase2 0x01-0x0F period of up to 15 DCLK clocks (reset 0x02, 0 is invalid)
    void setPrechargePeriod(uint8_t phase1, uint8_t phase2);
    
    // See defines above for levels
    void setVcomhDeselectLevel(uint8_t level);


    // Command for no-operation
    void nop();
    

/** @brief Horizontal scroll by one column per interval
 *  @param bool left select Left/Right scroll
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t interval   scroll interval in frames (see codes above)                      
 */  
    void setContinuousHorizontalScroll(bool left, uint8_t start_page, uint8_t end_page, uint8_t interval);


/** @brief Horizontal and Vertical scroll by one column per interval
 *  @param bool left select Left/Right scroll
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t offset     vert offset  (0x01..0x63)                       
 *  @param uint8_t interval   scroll interval in frames (see codes above)                       
 */  
    void setContinuousVerticalAndHorizontalScroll(bool left, uint8_t start_page, uint8_t end_page, 
                                                  uint8_t offset, uint8_t interval);
    
/** @brief Activate or Deactivate Horizontal and Vertical scroll
 *  @brief Note: after deactivating scrolling, the RAM data needs to be rewritten
 *  @param bool on activate scroll 
 */  
    void setDisplayScroll(bool on);
    

/** @brief Set Vertical scroll area
 *  @param uint8_t topRowsFixed      fixed rows   (0..MAX_ROW)                     
 *  @param uint8_t scrollRowsoffset  scroll rows  (topRowsFixed..MAX_ROW)                       
 */  
    void setVerticalScrollArea(uint8_t topRowsFixed, uint8_t scrollRows); 
  
  private:

// Low Level methods

/** @brief Write command that has no parameters
*/    
    void _sendCommand(uint8_t command); 

/** @brief Write command that has one parameter
*/    
    void _sendCommand(uint8_t command, uint8_t param1);

/** @brief Write command that has two parameters
*/ 
    void _sendCommand(uint8_t command, uint8_t param1, uint8_t param2);              
//    void sendCommands(uint8_t len, uint8_t* buf);

/** @brief Write command that has five parameters
*/ 
    void _sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                       uint8_t param3, uint8_t param4,
                                       uint8_t param5);

/** @brief Write command that has six parameters
*/ 
    void _sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                       uint8_t param3, uint8_t param4,
                                       uint8_t param5, uint8_t param6);
    
/** @brief Write databyte to display
 *  @brief Start at current cursor location
 *  @param uint8_t data databyte to write
*/  
    void _sendData(uint8_t data);

/** @brief Write len bytes from buffer data to display, 
 *  @brief Start at current cursor location
 *  @param uint8_t len number of bytes to write 
 *  @param uint8_t* data pointer to data
*/   
    void _sendData(uint8_t len, uint8_t* data);

/** @brief Low level Init
 *  @brief Init the configuration registers in accordance with the datasheet
 */   
    void _init();

    mbed::I2C *_i2c;             // I2C bus reference
    uint8_t _readOpcode;   // contains the I2C address of the device
    uint8_t _writeOpcode;  // contains the I2C address of the device
    
    bool _inverted;        // inverted or normal text   
};

#endif
