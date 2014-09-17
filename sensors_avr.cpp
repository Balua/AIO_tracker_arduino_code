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
#include "sensors_avr.h"
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif



#include <SoftwareSerial.h>
#include <Wire.h>


#include <OneWire.h>
#include <DallasTemperature.h>

#ifdef SOFTSERIALDEBUG
extern SoftwareSerial softdebug;
#endif

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire temp_sensor_wire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&temp_sensor_wire);
// arrays to hold device address
DeviceAddress insideThermometer = { 0x28, 0x36, 0x91, 0x8E, 0x05, 0x00, 0x00, 0xC2 };

int sensors_vin()
{
  analogReference(DEFAULT);      // Ref=3.3V
  analogRead(VMETER_PIN);        // Disregard the 1st conversion after changing ref (p.256)
  delay(10);                     // This is needed when switching references

  uint16_t adc = analogRead(VMETER_PIN); 
  uint16_t mV = 3300L * adc / 1024;
   
  // Vin = mV * R2 / (R1 + R2)
  int vin = (uint32_t)mV * (VMETER_R1 + VMETER_R2) / VMETER_R2;
  return vin;
}


void getdata(byte *a, byte *b)
{
  Wire.requestFrom(sensor,2);//Sends content of first two registers
  *a = Wire.read(); //first byte recieved stored here
  *b = Wire.read(); //second byte recieved stored here
}

long sensors_pressure(){
  
  byte aa,bb;
  float pressure =0;
  getdata(&aa,&bb);
  float c = aa*256+bb;//combines byte 1 and byte 2
  pressure= ((c-1638)*1.0342)/(14745-1638)*100000;// Conversion found from technical notes/ pressure is in bar
  
  const float p0 = 101325;     // Pressure at sea level (Pa)
  long press_alt = (float)44330.0 * (1.0 - pow(((float) pressure/p0), 0.190295)); //convert pressure value in pressure altitude (m)
  
   
#ifdef DEBUG_SENS  
  softdebug.println();
  softdebug.print("Pressure:");
  softdebug.print(pressure);
  softdebug.println(" Pa");
  softdebug.print("Pressure Altitude");
  softdebug.print(press_alt);
  softdebug.println("m");
#endif
    
  return press_alt;
}



int sensors_temperature(){
    
// call temp_sensor.requestTemperatures() to issue a global temperature request to all devices on the bus
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
// call temp_sensor.getTempC to read temperature in degrees Celsius from the device
  int temperature =(int) temp_sensor.getTempC(insideThermometer);
 
#ifdef DEBUG_SENS  
  softdebug.println();
  softdebug.print("Temp C: ");
  softdebug.println(temperature);
#endif
    
  return temperature;
}



void sensor_setup(){  
  temp_sensor.begin();//initialize one wire sensor
  temp_sensor.setResolution(insideThermometer, 9);  //configure sensor parameters 
  
   
#ifdef DEBUG_SENS
  // report on finding the devices on the bus or not
  if (!temp_sensor.getAddress(insideThermometer, 0)) softdebug.println("Unable to find address for Device 0");
  else{  
    
    // report parasite power requirements
    softdebug.print("Parasite power is: "); 
    if (temp_sensor.isParasitePowerMode()) softdebug.println("ON");
    else softdebug.println("OFF");
    
    sensors_temperature(); //print temperature for debugging
    
  }
  
  //print voltage sensor value for debugging
  softdebug.print("Vin=");
  softdebug.println(sensors_vin());
  
#endif //DEBUG_SENS
}







#endif // ifdef AVR
