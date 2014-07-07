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

// Mpide 22 fails to compile Arduino code because it stupidly defines ARDUINO 
// as an empty macro (hence the +0 hack). UNO32 builds are fine. Just use the
// real Arduino IDE for Arduino builds. Optionally complain to the Mpide
// authors to fix the broken macro.
#if (ARDUINO + 0) == 0
#error "Oops! We need the real Arduino IDE (version 22 or 23) for Arduino builds."
#error "See trackuino.pde for details on this"

// Refuse to compile on arduino version 21 or lower. 22 includes an 
// optimization of the USART code that is critical for real-time operation
// of the AVR code.
#elif (ARDUINO + 0) < 22
#error "Oops! We need Arduino 22 or 23"
#error "See trackuino.pde for details on this"

#endif


// Trackuino custom libs
#include "config.h"
#include "afsk_avr.h"
#include "aprs.h"
#include "gps.h"
#include "pin.h"
#include "power.h"
#include "sensors_avr.h"

// Arduino/AVR libs
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Module constants
static const uint32_t VALID_POS_TIMEOUT = 2000;  // ms

// Module variables
static int32_t next_aprs = 0;

//initialize tinygps object named ublox  
TinyGPS ublox;

//configure softserial port for debuging purposes
#ifdef SOFTSERIALDEBUG
SoftwareSerial softdebug(A2, A3);
#endif


void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pin_write(LED_PIN, LOW);

  pinMode(4, OUTPUT);
  pin_write(4, HIGH); // enable all 5V electronics by enabling 5volt regulator

#ifdef SOFTSERIALDEBUG  
  softdebug.begin(SOFTSERIALDEBUG_BAUDRATE);
  softdebug.println("Soft serial port open");
#endif

#ifdef DEBUG_RESET
  softdebug.println("RESET");
#endif
  gps_setup();
  afsk_setup();

#ifdef DEBUG_SENS
  softdebug.print(", Vin=");
  softdebug.println(sensors_vin());
#endif

  // Do not start until we get a valid time reference
  // for slotted transmissions.
  if (APRS_SLOT >= 0) {
    do {
      while (! Serial.available())  ///enquanto nao houver bytes na porta serial - > sem sinal GPS
        power_save();  //entrar em power save ate ao sinal seguinte...
    } while (! ublox.encode(Serial.read()));//ir acordando e adormecendo enquanto nao se conseguir descodificar uma frase completa
    
    next_aprs = millis() + 1000 *
      (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
  }
  else {
    next_aprs = millis();
  }  

}

void get_pos()
{
#ifdef DEBUG_GPS
      softdebug.println("entering get_pos()");    
#endif
  char c;
  // Get a valid position from the GPS
  int valid_pos = 0;
  uint32_t timeout = millis();
  do {
    if (Serial.available()){
      c=Serial.read();
      valid_pos = ublox.encode(c);
#ifdef DEBUG_GPS
      softdebug.print(c);
#endif
    }
  } while ( (millis() - timeout < VALID_POS_TIMEOUT) && ! valid_pos) ; 
  // stop loop if valid position=TRUE or if defined timeout is reached (definition on AIO_Tracker_arduino_code.ino)
  if (valid_pos){
   ublox_to_aprs(); 
  }
#ifdef DEBUG_GPS
      softdebug.println("exiting get_pos()");    
#endif
}

void loop()
{  
  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {
    get_pos();
    aprs_send();
    next_aprs += APRS_PERIOD * 1000L;
    while (afsk_flush()) {
      power_save();
    }

#ifdef DEBUG_MODEM
    // Show modem ISR stats from the previous transmission
    afsk_debug();
#endif
  }

  power_save(); // Incoming GPS data or interrupts will wake us up
}
