/**  @file SSD1308 I2C device class file
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
#include "mbed.h"
#include "SSD1308.hpp"

//#include "font_3x5.h"
//#include "font_5x7.h"
//#include "font_6x8.h"
#include "font_8x8.h"
//#include "font_8x12.h"
//#include "font_16x20.h"
#include "font_16x24.h"

#if defined(TARGET_LPC1768)
#define I2C_OPTIMIZE   1
#else
#define I2C_OPTIMIZE   0
#endif

/**
 *@brief Constructor
 *@param I2C *i2c reference to i2c
 *@param uint8_t deviceAddress slaveaddress
 */
SSD1308::SSD1308(I2C *i2c, uint8_t deviceAddress) : _i2c(i2c) {
  
  _writeOpcode = deviceAddress & 0xFE; // low order bit = 0 for write
  _readOpcode  = deviceAddress | 0x01; // low order bit = 1 for read  
  
  initialize(); 
}

/** @brief High level Init, most settings remain at Power-On reset value
 */
void SSD1308::initialize() {
  setHorizontalAddressingMode();

  clearDisplay();

  setInverted(false);
  
  setDisplayOn();  
}


/** @brief clear the display
*/
#if (I2C_OPTIMIZE == 0)
// Standard version
void SSD1308::clearDisplay() {
 
  //setDisplayOff();
  setPageAddress(0, MAX_PAGE);  // all pages
  setColumnAddress(0, MAX_COL); // all columns

  for (uint8_t page = 0; page < PAGES; page++) {
    for (uint8_t col = 0; col < COLUMNS; col++) {
      _sendData(0x00);
    }
  }

  //setDisplayOn();
}
#else
//Optimised version
// Save lots of I2C S,P, address and datacommands:
// Send S, address, DATA_MODE, data, data, data,...., P
//
void SSD1308::clearDisplay() {

  //setDisplayOff();
  
  setPageAddress(0, MAX_PAGE);  // all pages
  setColumnAddress(0, MAX_COL); // all columns

  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  for (int i=0; i<(PAGES * COLUMNS); i++) {
    _i2c->write(0x00);  // Write Data   
  }
  _i2c->stop();

  //setDisplayOn();
}
#endif


/** @brief fill the display
 *  @param uint8_t pattern fillpattern vertical patch or 8 bits 
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t start_col  begin column (0..MAX_COL)
 *  @param uint8_t end_col    end column   (start_col..MAX_COL)
*/
#if (I2C_OPTIMIZE == 0)
//Standard version

void SSD1308::fillDisplay(uint8_t pattern,
                          uint8_t start_page, uint8_t end_page,
                          uint8_t start_col, uint8_t end_col) {
  
  int count = (end_page - start_page + 1) * (end_col - start_col + 1);
  
  //setDisplayOff();
  setPageAddress(start_page, end_page);  // set page window
  setColumnAddress(start_col, end_col);  // set column window
 
  for (int i=0; i<count; i++) {
    _sendData(pattern); // Write Data    
  }

  //setDisplayOn();
}

#else

//Optimised version
// Save lots of I2C S,P, address and datacommands:
// Send S, address, DATA_MODE, data, data, data,...., P
//
void SSD1308::fillDisplay(uint8_t pattern,
                          uint8_t start_page, uint8_t end_page,
                          uint8_t start_col, uint8_t end_col) {
  
  int count = (end_page - start_page + 1) * (end_col - start_col + 1);
  
  //setDisplayOff();
  setPageAddress(start_page, end_page);  // set page window
  setColumnAddress(start_col, end_col);  // set column window
 
  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  for (int i=0; i<count; i++) {
    _i2c->write(pattern);  // Write Data   
  }
  _i2c->stop();

  //setDisplayOn();
}

#endif


/** @brief write a bitmap to the display
 *  @param uint8_t* data pointer to bitmap
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t start_col  begin column (0..MAX_COL)
 *  @param uint8_t end_col    end column   (start_col..MAX_COL)
*/
#if (I2C_OPTIMIZE == 0)
//Standard version
void SSD1308::writeBitmap(uint8_t* data,
                          uint8_t start_page, uint8_t end_page,
                          uint8_t start_col, uint8_t end_col){
  
  int count = (end_page - start_page + 1) * (end_col - start_col + 1);

  //setDisplayOff();
  setPageAddress(start_page, end_page);  // set page window
  setColumnAddress(start_col, end_col);  // set column window

  for (int i=0; i<count; i++) {
    _sendData(data[i]); // Write Data   
  }

  //setDisplayOn();
}

#else
//Optimised version
// Save lots of I2C S,P, address and datacommands:
// Send S, address, DATA_MODE, data, data, data,...., P
//
void SSD1308::writeBitmap(uint8_t* data,
                          uint8_t start_page, uint8_t end_page,
                          uint8_t start_col, uint8_t end_col){
  
  int count = (end_page - start_page + 1) * (end_col - start_col + 1);

  //setDisplayOff();
  setPageAddress(start_page, end_page);  // set page window
  setColumnAddress(start_col, end_col);  // set column window

  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  for (int i=0; i<count; i++) {
    _i2c->write(data[i]);  // Write Data       
  }
  _i2c->stop();

  //setDisplayOn();
}

#endif


/** @brief write a progressbar to the display, Width is (PRG_MAX_SCALE + 2) pixels
 *  @param uint8_t page begin page   (0..MAX_PAGE)
 *  @param uint8_t col  begin column (0..MAX_COL)
 *  @param int percentage value      (0..100)
*/
#define PRG_MAX_SCALE     50
#define PRG_LEFT_EDGE   0xFF
#define PRG_RIGHT_EDGE  0xFF
#define PRG_ACTIVE      0xFF
//#define PRG_ACTIVE      0xBD
#define PRG_NOT_ACTIVE  0x81

#if (I2C_OPTIMIZE == 0)
//Standard version
void SSD1308::writeProgressBar(uint8_t page, uint8_t col, int percentage) {
  uint8_t scale_value;
  
  if (percentage <= 0) {
    scale_value = 0;
  } else if (percentage >= 100) {
      scale_value = PRG_MAX_SCALE - 1;
  }
  else {
    scale_value = (percentage * PRG_MAX_SCALE) / 100; 
  }      
      
  //setDisplayOff();
  setPageAddress(page, page);  
  setColumnAddress(col, MAX_COL); 
  
  _sendData(PRG_LEFT_EDGE);

  for (uint8_t col = 0; col < scale_value; col++) {
      _sendData(PRG_ACTIVE);
  }
      
  _sendData(PRG_ACTIVE);
  
  for (uint8_t col = (scale_value+1); col < PRG_MAX_SCALE; col++) {
      _sendData(PRG_NOT_ACTIVE);
  }

  _sendData(PRG_RIGHT_EDGE);    
  
  //setDisplayOn();
}
#else

//Optimised version
// Save lots of I2C S,P, address and datacommands:
// Send S, address, DATA_MODE, data, data, data,...., P
//
void SSD1308::writeProgressBar(uint8_t page, uint8_t col, int percentage) {
  uint8_t scale_value;
  
  if (percentage <= 0) {
    scale_value = 0;
  } else if (percentage >= 100) {
      scale_value = PRG_MAX_SCALE - 1 ;
  }
  else {
    scale_value = (percentage * PRG_MAX_SCALE) / 100; 
  }      
      
  //setDisplayOff();
  setPageAddress(page, page);  
  setColumnAddress(col, MAX_COL); 

  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  

  _i2c->write(PRG_LEFT_EDGE);  // Write Data         

  for (uint8_t col = 0; col < scale_value; col++) {
     _i2c->write(PRG_ACTIVE);  // Write Data                       
  }

  _i2c->write(PRG_ACTIVE);  // Write Data                       
     
  for (uint8_t col = (scale_value+1); col < PRG_MAX_SCALE; col++) {
     _i2c->write(PRG_NOT_ACTIVE);  // Write Data                 
  }

  _i2c->write(PRG_RIGHT_EDGE);  // Write Data           

  _i2c->stop();
    
  //setDisplayOn();
}
#endif

/** @brief write a level meter to the display, Width is (PRG_MAX_SCALE + 2) pixels
 *  @param uint8_t page begin page   (0..MAX_PAGE)
 *  @param uint8_t col  begin column (0..MAX_COL)
 *  @param int percentage value      (0..100)
*/
#if (I2C_OPTIMIZE == 0)
void SSD1308::writeLevelBar(uint8_t page, uint8_t col, int percentage) {
  uint8_t scale_value;
  
  if (percentage <= 0) {
    scale_value = 0;
  } else if (percentage >= 100) {
      scale_value = PRG_MAX_SCALE - 1;
  }
  else {
    scale_value = (percentage * PRG_MAX_SCALE) / 100; 
  }      
      
  //setDisplayOff();
  setPageAddress(page, page);  
  setColumnAddress(col, MAX_COL); 
 
  _sendData(PRG_LEFT_EDGE);   

  for (uint8_t col = 0; col < scale_value; col++) {
     _sendData(PRG_NOT_ACTIVE);  // Write Data                       
  }

  _sendData(PRG_ACTIVE);  // Write Data at active meterlevel

  for (uint8_t col = scale_value+1; col < PRG_MAX_SCALE; col++) {
      _sendData(PRG_NOT_ACTIVE);                
  }
         
  _sendData(PRG_RIGHT_EDGE);
    
  //setDisplayOn();
}
#else
//Optimised version
// Save lots of I2C S,P, address and datacommands:
// Send S, address, DATA_MODE, data, data, data,...., P
//
void SSD1308::writeLevelBar(uint8_t page, uint8_t col, int percentage) {
  uint8_t scale_value;
  
  if (percentage <= 0) {
    scale_value = 0;
  } else if (percentage >= 100) {
      scale_value = PRG_MAX_SCALE - 1;
  }
  else {
    scale_value = (percentage * PRG_MAX_SCALE) / 100; 
  }      
      
  //setDisplayOff();
  setPageAddress(page, page);  
  setColumnAddress(col, MAX_COL); 

  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  

  _i2c->write(PRG_LEFT_EDGE);  // Write Data         

  for (uint8_t col = 0; col < scale_value; col++) {
     _i2c->write(PRG_NOT_ACTIVE);  // Write Data                       
  }

  _i2c->write(PRG_ACTIVE);  // Write Data at active meterlevel
  
  for (uint8_t col = scale_value+1; col < PRG_MAX_SCALE; col++) {
     _i2c->write(PRG_NOT_ACTIVE);  // Write Data                 
  }

  _i2c->write(PRG_RIGHT_EDGE);  // Write Data           

  _i2c->stop();
    
  //setDisplayOn();
}
#endif

/** @brief Write single character to the display using the 8x8 fontable
 *  @brief Start at current cursor location
 *  @param char chr character to write
*/
void SSD1308::writeChar(char chr) {

  const uint8_t char_index = chr - 0x20;

  for (uint8_t i = 0; i < 8; i++) {
     if (_inverted) {
       _sendData( ~font_8x8[char_index][i] );           
     }
     else {
       _sendData( font_8x8[char_index][i] );
     }  
  }

}


/** @brief Write a string to the display using the 8x8 font
 *  @brief Start at selected cursor location, text will wrap around until it is done
 *  @param uint8_t row  row number    (0...ROWS/FONT_HEIGHT)
 *  @param uint8_t col  column number (0...COLUMNS/FONT_WIDTH)
 *  @param const char * text pointer to text
 */
void SSD1308::writeString(uint8_t row, uint8_t col, const char * text) {
  uint16_t index = 0;
  uint16_t len = strlen(text);
  
  setPageAddress(row, MAX_PAGE);
  const uint8_t col_addr = FONT8x8_WIDTH*col;
  setColumnAddress(col_addr, MAX_COL);

  while ((col+index) < CHARS && (index < len)) {
     // write first line, starting at given position
     writeChar(text[index++]);
  }

  // write remaining lines
  // write until the end of memory
  // then wrap around again from the top.
  if (index + 1 < len) {
    setPageAddress(row + 1, MAX_PAGE);
    setColumnAddress(0, MAX_COL);
    bool wrapEntireScreen = false;
    while (index + 1 < len) {
       writeChar(text[index++]);
       // if we've written the last character space on the screen, 
       // reset the page and column address so that it wraps around from the top again
       if (!wrapEntireScreen && (row*CHARS + col + index) > 127) {
         setPageAddress(0, MAX_PAGE);
         setColumnAddress(0, MAX_COL);
         wrapEntireScreen = true;
       }
    }
  }
}



/** @brief Write large character (16x24 font)
 *  @param uint8_t row  row number    (0...MAX_ROW)
 *  @param uint8_t col  column number (0...MAX_COL)
 *  @param char chr     Used for displaying numbers 0 - 9 and '+', '-', '.'
 */
void SSD1308::writeBigChar(uint8_t row, uint8_t col, char chr) {

  writeBitmap((uint8_t*) font_16x24[int(chr) - FONT16x24_START],
              row, (row + FONT16x24_BYTES - 1),
              col, (col + FONT16x24_WIDTH - 1));
}


/** @brief Write command that has no parameters
*/ 
void SSD1308::_sendCommand(uint8_t command) {
//  I2Cdev::writeByte(m_devAddr, COMMAND_MODE, command);

#if (I2C_OPTIMIZE == 0)
  char databytes[2];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  _i2c->write(_writeOpcode, databytes, 2);    // Write command   
#else  

  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   

  _i2c->stop();  
#endif
}

/** @brief Write command that has one parameter
*/ 
void SSD1308::_sendCommand(uint8_t command, uint8_t param1) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[4];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  _i2c->write(_writeOpcode, databytes, 4);    // Write command   
#else  

  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   

  _i2c->stop();
#endif  
}

/** @brief Write command that has two parameters
*/ 
void SSD1308::_sendCommand(uint8_t command, uint8_t param1, uint8_t param2) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[6];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  databytes[4] = COMMAND_MODE;
  databytes[5] = param2; 
  _i2c->write(_writeOpcode, databytes, 6);    // Write command   
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param2);        // Write Param2   

  _i2c->stop();
 #endif 
}

/** @brief Write command that has five parameters
*/ 
void SSD1308::_sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                            uint8_t param3, uint8_t param4,
                                            uint8_t param5) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[12];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  databytes[4] = COMMAND_MODE;
  databytes[5] = param2; 
  databytes[6] = COMMAND_MODE;
  databytes[7] = param3; 
  databytes[8] = COMMAND_MODE;
  databytes[9] = param4; 
  databytes[10] = COMMAND_MODE;
  databytes[11] = param5;       
  _i2c->write(_writeOpcode, databytes, 12);    // Write command   
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param2);        // Write Param2   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param3);        // Write Param3   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param4);        // Write Param4   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param5);        // Write Param5   

  _i2c->stop();
#endif  
}


/** @brief Write command that has six parameters
*/ 
void SSD1308::_sendCommand(uint8_t command, uint8_t param1, uint8_t param2,
                                            uint8_t param3, uint8_t param4,
                                            uint8_t param5, uint8_t param6) {

//  Note continuationbit is set, so COMMAND_MODE must be
//  repeated before each databyte that serves as parameter!
#if (I2C_OPTIMIZE == 0)
  char databytes[14];
    
  databytes[0] = COMMAND_MODE;
  databytes[1] = command;    
  databytes[2] = COMMAND_MODE;
  databytes[3] = param1; 
  databytes[4] = COMMAND_MODE;
  databytes[5] = param2; 
  databytes[6] = COMMAND_MODE;
  databytes[7] = param3; 
  databytes[8] = COMMAND_MODE;
  databytes[9] = param4; 
  databytes[10] = COMMAND_MODE;
  databytes[11] = param5;   
  databytes[12] = COMMAND_MODE;
  databytes[13] = param6;       
  _i2c->write(_writeOpcode, databytes, 14);    // Write command   
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  
  _i2c->write(COMMAND_MODE);      
  _i2c->write(command);       // Write Command   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param1);        // Write Param1   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param2);        // Write Param2   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param3);        // Write Param3   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param4);        // Write Param4   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param5);        // Write Param5   
  _i2c->write(COMMAND_MODE);      
  _i2c->write(param6);        // Write Param6   

  _i2c->stop();
#endif  
}


#if(0)
/** @brief Write command that has multiple parameters
*/ 
void SSD1308::_sendCommands(uint8_t len, uint8_t* commands) {

//  I2Cdev::writeBytes(m_devAddr, COMMAND_MODE, len, commands);
//  Note this original code is not correct, continuationbit is set, 
//  so COMMAND_MODE must be repeated before each databyte that serves as parameter!

  _i2c->start();
  _i2c->write(_writeOpcode);
  
  for (int i=0; i<len ; i++) {
    _i2c->write(COMMAND_MODE);      
    _i2c->write(commands[i]);  // Write Commands   
  }
  _i2c->stop();
  
}
#endif

/** @brief Write databyte to display
 *  @brief Start at current cursor location
 *  @param uint8_t data databyte to write
*/
void SSD1308::_sendData(uint8_t data){

#if (I2C_OPTIMIZE == 0)
//I2C Blockwrite versions dont seem to work ?
//That may be related to fact that the SSD1308/SSD1306 does NOT return an acknowledge: blockwrite may abort the operation
//Noted for mbed lib v63 on 20/7/13 
  char databytes[2];
    
  databytes[0] = DATA_MODE;
  databytes[1] = data;    
  _i2c->write(_writeOpcode, databytes, 2);    // Write Data   

#else
  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  _i2c->write(data); 
  _i2c->stop();  
#endif

}

/** @brief Write len bytes from buffer data to display, 
 *  @brief Start at current cursor location
 *  @param uint8_t len number of bytes to write 
 *  @param uint8_t* data pointer to data
*/
void SSD1308::_sendData(uint8_t len, uint8_t* data) {
//  I2Cdev::writeBytes(m_devAddr, DATA_MODE, len, data);
#if (I2C_OPTIMIZE == 0)
  for (int i=0; i<len ; i++) {
    _sendData(data[i]);  // Write Data   
  }
#else  
  _i2c->start();
  _i2c->write(_writeOpcode);
  _i2c->write(DATA_MODE);  
  for (int i=0; i<len ; i++) {
    _i2c->write(data[i]);  // Write Data   
  }
  _i2c->stop();
#endif 
}


/** @brief Set Horizontal Addressing Mode (cursor incr left-to-right, top-to-bottom)
 * 
 */
void SSD1308::setHorizontalAddressingMode(){
  setMemoryAddressingMode(HORIZONTAL_ADDRESSING_MODE); 
}

/** @brief Set Vertical Addressing Mode  (cursor incr top-to-bottom, left-to-right)
 * 
 */
void SSD1308::setVerticalAddressingMode() {
  setMemoryAddressingMode(VERTICAL_ADDRESSING_MODE); 
}

/** @brief Set Page Addressing Mode  (cursor incr left-to-right)
 * 
 */
void SSD1308::setPageAddressingMode(){
  setMemoryAddressingMode(PAGE_ADDRESSING_MODE); 
}
    
/** @brief Set Addressing Mode
 *  @param uint8_t mode 
 */
void SSD1308::setMemoryAddressingMode(uint8_t mode){

  _sendCommand(SET_MEMORY_ADDRESSING_MODE, mode);   
}


/** @param uint8_t start startpage (valid range 0..MAX_PAGE)
 *  @param uint8_t end   endpage   (valid range start..MAX_PAGE)
 */
void SSD1308::setPageAddress(uint8_t start, uint8_t end) {

  _sendCommand(SET_PAGE_ADDRESS, start, end);   
}


/** @param uint8_t start startcolumn (valid range 0..MAX_COL)
 *  @param uint8_t end   endcolumn   (valid range start..MAX_COL)
 */
void SSD1308::setColumnAddress(uint8_t start, uint8_t end) {

  _sendCommand(SET_COLUMN_ADDRESS, start, end);     
}

/** 
 *  @brief Set Display StartLine, takes one byte, 0x00-0x3F
 *  @param uint8_t line startline (valid range 0..MAX_ROWS)
 */  
void SSD1308::setDisplayStartLine(uint8_t line) {

  line = line & MAX_ROW;
   
  _sendCommand(SET_DISPLAY_START_LINE | line);     
}


/** 
 *  @brief Set Column Start (for Page Addressing Mode only)
 *  @param uint8_t column column start (valid range 0..MAX_COL)
 */  
void SSD1308::setColumnStartForPageAddressingMode(uint8_t column) {

  column = column & MAX_COL;

  _sendCommand(SET_LOWER_COLUMN  | ( column     & 0x0F));  // lower nibble   
  _sendCommand(SET_HIGHER_COLUMN | ((column>>4) & 0x0F));  // higher nibble     
}


/** 
 *  @brief Set Page Start (for Page Addressing Mode only)
 *  @param uint8_t page page start (valid range PAGE0 - PAGE7)
 */    
void SSD1308::setPageStartForPageAddressingMode(uint8_t page) {

  page = page & MAX_PAGE;

  _sendCommand(SET_PAGE_START_ADDRESS | page);
 
}


/** @brief Set Contrast
 *  @param uint8_t contrast (valid range 0x00 (lowest) - 0xFF (highest))
*/
void SSD1308::setContrastControl(uint8_t contrast) {
  
    _sendCommand(SET_CONTRAST, contrast);  
} 

/** @brief Enable Display
*/ 
void SSD1308::setDisplayOn() {
  _sendCommand(SET_DISPLAY_POWER_ON);
}

/** @brief Disable Display
*/ 
void SSD1308::setDisplayOff() {
  _sendCommand(SET_DISPLAY_POWER_OFF);
}

/** @brief Enable or Disable Display
 *  @param bool on
 */
void SSD1308::setDisplayPower(bool on) {
  if (on) {
    setDisplayOn();
  } else {
    setDisplayOff();
  }
}

/** @brief Show White pixels on Black background
 */ 
void SSD1308::setDisplayNormal() {
  _sendCommand(SET_NORMAL_DISPLAY);
}

/** @brief Show Black pixels on White background
 */ 
void SSD1308::setDisplayInverse() {
  _sendCommand(SET_INVERSE_DISPLAY);
}

/** @brief Blink display by fading in and out over a set number of frames
 *  @param bool on
 */
void SSD1308::setDisplayBlink(bool on){
  if (on) {
    _sendCommand(SET_FADE_BLINK, (BLINK_ENABLE | FADE_INTERVAL_128_FRAMES));
  }
  else {  
    _sendCommand(SET_FADE_BLINK, FADE_BLINK_DISABLE);  
  }
}       


/** @brief Fade out display in set number of frames
 *  @param bool on
 */
void SSD1308::setDisplayFade(bool on) {
  if (on) {
    _sendCommand(SET_FADE_BLINK, (FADE_OUT_ENABLE | FADE_INTERVAL_128_FRAMES));
  }
  else {  
    _sendCommand(SET_FADE_BLINK, FADE_BLINK_DISABLE);  
  }
}    

/** @brief Display Flip (Left/Right, Up/Down)
 *  @param bool left flip Left/Right
 *  @param bool down flip Up/Down
 */
void SSD1308::setDisplayFlip(bool left, bool down) {
  if (left) {
    // column address   0 is mapped to SEG0 (Reset)    
    _sendCommand(SET_SEGMENT_REMAP_0);
  }
  else {
    // column address 127 is mapped to SEG0    
    _sendCommand(SET_SEGMENT_REMAP_127);
  }  

  if (down) {
    // Reset mode
    _sendCommand(SET_COMMON_REMAP_0);    
  }
  else {
    // Flip Up/Down (Need to rewrite display before H effect shows)
    _sendCommand(SET_COMMON_REMAP_63);        
  }  

}

/** @brief Sets Internal Iref
 */
void SSD1308::setInternalIref() {
//  uint8_t cmds[2] = {SET_IREF_SELECTION, INTERNAL_IREF};
//  _sendCommands(2, cmds); 
  
  _sendCommand(SET_IREF_SELECTION, INTERNAL_IREF);   
}

/** @brief Sets External Iref (default)
 */
void SSD1308::setExternalIref() {
//  uint8_t cmds[2] = {SET_IREF_SELECTION, EXTERNAL_IREF};
//  _sendCommands(2, cmds); 
  _sendCommand(SET_IREF_SELECTION, EXTERNAL_IREF); 
}


/** @brief Shows All Pixels On
 */
void SSD1308::setEntireDisplayOn(){
  _sendCommand(SET_ENTIRE_DISPLAY_ON); 
}

/** @brief Shows Pixels as RAM content
 */
void SSD1308::setEntireDisplayRAM(){
  _sendCommand(SET_DISPLAY_GDDRAM); 
}

/** @brief Shows Pixels On or as RAM content
 *  @param bool on (true is All on, false is RAM content)
 */
void SSD1308::setEntireDisplay(bool on){
  if (on) {
    setEntireDisplayOn();  // All Pixels on
  }
  else {  
    setEntireDisplayRAM(); // Pixels are RAM content
  }
}


/** @brief Horizontal scroll by one column per interval
 *  @param bool left select Left/Right scroll
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t interval   scroll interval in frames (see codes above)                      
 */  
void SSD1308::setContinuousHorizontalScroll(bool left, uint8_t start_page, uint8_t end_page, uint8_t interval) {
  if (left) {
    _sendCommand(SET_LEFT_HOR_SCROLL, 0x00, start_page, interval, end_page, 0x00, 0xFF);  // Scroll Left
  }
  else {  
    _sendCommand(SET_RIGHT_HOR_SCROLL, 0x00, start_page, interval, end_page, 0x00, 0xFF); // Scroll Right  
  }

}


/** @brief Horizontal and Vertical scroll by one column per interval
 *  @param bool left select Left/Right scroll
 *  @param uint8_t start_page begin page   (0..MAX_PAGE)
 *  @param uint8_t end_page   end page     (start_page..MAX_PAGE)                     
 *  @param uint8_t offset     vert offset  (0x01..0x63)                       
 *  @param uint8_t interval   scroll interval in frames (see codes above)                       
 */  
void SSD1308::setContinuousVerticalAndHorizontalScroll(bool left, uint8_t start_page, uint8_t end_page, 
                                                       uint8_t offset, uint8_t interval) {
  if (left) {
    _sendCommand(SET_VERT_LEFT_HOR_SCROLL, 0x00, start_page, interval, end_page, offset);  // Scroll Left
  }
  else {  
    _sendCommand(SET_VERT_RIGHT_HOR_SCROLL, 0x00, start_page, interval, end_page, offset); // Scroll Right  
  }
                                                       
}    

/** @brief Set Vertical scroll area
 *  @param uint8_t topRowsFixed      fixed rows   (0..MAX_ROW)                     
 *  @param uint8_t scrollRowsoffset  scroll rows  (topRowsFixed..ROWS)                       
 */  
void SSD1308::setVerticalScrollArea(uint8_t topRowsFixed, uint8_t scrollRows) { 
   
  if ((topRowsFixed + scrollRows) > ROWS) {
     scrollRows = ROWS - topRowsFixed; 
  };
  
  _sendCommand(SET_VERTICAL_SCROLL_AREA, topRowsFixed, scrollRows); 
}

/** @brief Activate or Deactivate Horizontal and Vertical scroll
 *  @brief Note: after deactivating scrolling, the RAM data needs to be rewritten
 *  @param bool on activate scroll 
 */  
void SSD1308::setDisplayScroll(bool on) {
  if (on) {
    _sendCommand(SET_ACTIVATE_SCROLL);   // Scroll on
  }
  else {  
    _sendCommand(SET_DEACTIVATE_SCROLL); // Scroll off  
  }
}



/** @brief Low level Init
 *  @brief Init the configuration registers in accordance with the datasheet
 */
void SSD1308::_init() {

  _sendCommand(SET_DISPLAY_POWER_OFF);      // 0xAE
  
  // column address   0 is mapped to SEG0 (Reset)    
  // row address   0 is mapped to COM0 (Reset)      
  _sendCommand(SET_SEGMENT_REMAP_0);        // 0xA0 (Reset)
  _sendCommand(SET_COMMON_REMAP_0);         // 0xC0 (Reset) 

  setDisplayStartLine(0);                   // 0x40 (Reset) 
  
  _sendCommand(SET_COMMON_CONF, COMMON_BASE | COMMON_ALTERNATIVE | COMMON_LEFTRIGHT_NORMAL); // 0xDA, 0x12 (Reset)

  // Pagemode or Horizontal mode
//  setPageAddressingMode();                  // 0x20, 0x02 (Reset)  
//  setColumnStartForPageAddressingMode(0);   // 0x00, 0x10 (Reset = Column 0)
//  setPageStartForPageAddressingMode(PAGE_0);// 0xBO       (Reset = Page 0)
  setHorizontalAddressingMode();            // 0x20, 0x00 (Non-Reset)
  setColumnAddress(0, MAX_COL);             // 0x21, 0x00, 0x37 (Reset)
  setPageAddress(0, MAX_PAGE);              // 0x22, 0x00, 0x07 (Reset)

  setExternalIref();                        // 0xAD, 0x10 (Reset)
  
  _sendCommand(SET_DISPLAY_CLOCK, 0x70);    // 0xD5, 0x70 (Reset = 0x80)
  _sendCommand(SET_PRECHARGE_TIME, 0x21);   // 0xD9, 0x21 (Reset = 0x22)
  _sendCommand(SET_VCOMH_DESELECT_LEVEL, 0x30); // 0xDB, 0x30 (Reset = 0x20)  
  _sendCommand(SET_MULTIPLEX_RATIO, 0x3F);  // 0xA8, 0x3F (Reset)  
  _sendCommand(SET_DISPLAY_OFFSET, 0x00);   // 0xD3, 0x00 (Reset)  
  
  _sendCommand(SET_CONTRAST, 0x7F);         // 0x81, 0x7F (Reset)

  _sendCommand(SET_NORMAL_DISPLAY);         // 0xA6 (Reset)
  
  setEntireDisplayRAM();                    // 0xA4 (Reset)
  setDisplayScroll(false);
  
  clearDisplay();   
  
  _sendCommand(SET_DISPLAY_POWER_ON);       // 0xAF
}

