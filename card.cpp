/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* Credit to:
 *
 * cathedrow for this idea on using the ADC as a volt meter:
 * http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 */

#ifdef AVR

#include "config.h"
#include "pin.h"
#include "card.h"
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif

#include <SoftwareSerial.h>
#include <SD.h>

#ifdef SOFTSERIALDEBUG
extern SoftwareSerial softdebug;
#endif

#define CARD_CHIP_SELECT 8 //our SD card chip select is connected to digital pin 8


int sdcard_setup(){
  

#ifdef DEBUG_SD_CARD  
  softdebug.println();
  softdebug.println("Initializing SD card...");
#endif
  
  pinMode(10, OUTPUT);
  
  if (!SD.begin(CARD_CHIP_SELECT)) {
    
    #ifdef DEBUG_SD_CARD 
    softdebug.println("Card failed, or not present");
    #endif
    // don't do anything more:
    return false;
  }
  #ifdef DEBUG_SD_CARD 
  softdebug.println("card initialized.");
  #endif
  //File dataFile = SD.open("datalog.txt", FILE_WRITE);

  
}


void sdcard_write(){
  

  
}

#endif // ifdef AVR


