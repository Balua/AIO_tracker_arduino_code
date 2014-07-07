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

#include "config.h"
#include "gps.h"
#include "pin.h"
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif
#include <TinyGPS.h>
#include <stdlib.h>
#include <SoftwareSerial.h>

extern TinyGPS ublox;

#ifdef SOFTSERIALDEBUG
extern SoftwareSerial softdebug;
#endif



// Public (extern) variables, readable from other modules
char gps_time[7];       // HHMMSS
//uint32_t gps_seconds = 0;   // seconds after midnight
float gps_lat = 0;
float gps_lon = 0;
char gps_aprs_lat[9];
char gps_aprs_lon[10];
float gps_course = 0;
float gps_speed = 0;
float gps_altitude = 0;
unsigned int gps_fix_age = 0;

void gps_setup() {
  strcpy(gps_time, "000000");
  strcpy(gps_aprs_lat, "0000.00N");
  strcpy(gps_aprs_lon, "00000.00E"); 
  
  //enable the digital pin where the uBlox gps receiver's enable is connected. 
//force LOW to activate GPS
  pinMode(GPSEN_PIN, OUTPUT);
  pin_write(GPSEN_PIN, LOW);
  
  Serial.begin(GPS_BAUDRATE);
  
#ifdef DEBUG_GPS
      softdebug.println("GPS SETUP FINISH");
#endif
}


void ublox_to_aprs(){
  
#ifdef DEBUG_GPS
      softdebug.println("entering ublox_to_aprs()");    
#endif
  
  char temp_buffer[9];
  
  unsigned long fix_age, time, date;
 
  // time in hhmmsscc, date in ddmmyy
  ublox.get_datetime(&date, &time, &fix_age);
  ltoa(time,temp_buffer,10); //convert gps time into char array
  strncpy(gps_time,temp_buffer,6); //copy gps time chars to correct string eliminating the last two hundredths of second digits
  gps_time[6] = '\0';
 
  // returns +/- latitude/longitude in degrees
  ublox.f_get_position(&gps_lat, &gps_lon, &fix_age);
 
  //convert gps_lat into aprs compatible char array
  if (gps_lat >= 0){ //negative latitude = northern hemisphere "0000.00N"
    int lat_deg=(int)(gps_lat + 0.5);   //degrees in integer format
    float temp_lat_min=(((gps_lat-lat_deg)*60)+0.5);
    int lat_min=(int)temp_lat_min;    //minutes in integer format
    int lat_dec_min=(int)(((temp_lat_min-lat_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lat, "%02d%02d.%02dN", lat_deg,lat_min,lat_dec_min);
    gps_aprs_lat[8]='\0';
  }
  else{   //positive latitude = southern hemisphere "0000.00N"
    int lat_deg=(int)(-gps_lat + 0.5);   //degrees in integer format
    float temp_lat_min=(((-gps_lat-lat_deg)*60)+0.5);
    int lat_min=(int)temp_lat_min;    //minutes in integer format
    int lat_dec_min=(int)(((temp_lat_min-lat_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lat, "%02d%02d.%02dS", lat_deg,lat_min,lat_dec_min);
    gps_aprs_lat[8]='\0';
  }
  
  //convert gps_lon into aprs compatible char array
  if (gps_lon <= 0){ //negative longitude = western hemisphere "00000.00W"
    int lon_deg=(int)(-gps_lon + 0.5);   //degrees in integer format
    float temp_lon_min=(((-gps_lon-lon_deg)*60)+0.5);
    int lon_min=(int)temp_lon_min;    //minutes in integer format
    int lon_dec_min=(int)(((temp_lon_min-lon_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lon, "%03d%02d.%02dW", lon_deg,lon_min,lon_dec_min);
    gps_aprs_lon[9]='\0';
  }
  else{   //positive longitude = eastern hemisphere "00000.00E"
    int lon_deg=(int)(gps_lon + 0.5);   //degrees in integer format
    float temp_lon_min=(((gps_lon-lon_deg)*60)+0.5);
    int lon_min=(int)temp_lon_min;    //minutes in integer format
    int lon_dec_min=(int)(((temp_lon_min-lon_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lon, "%03d%02d.%02dE", lon_deg,lon_min,lon_dec_min);
    gps_aprs_lon[9]='\0';
  }
    
  // +/- altitude in meters
  gps_altitude = ublox.f_altitude(); 
  // course in degrees
  gps_course = ublox.f_course(); 
  // speed in km/hr
  gps_speed = ublox.f_speed_kmph(); 
  // Last Valid Position in seconds
  gps_fix_age=(unsigned int) fix_age/1000;

  
  #ifdef DEBUG_GPS
      softdebug.println("GPS Data: Time Lat Long APRS_Lat APRS_Long Alt Crs Spd LVP");  //Crs=course, Spd=speed,LVP=last valid position
      softdebug.print(gps_lat);
      softdebug.print(" ");
      softdebug.print(gps_lon);
      softdebug.print(" ");
      softdebug.print(gps_aprs_lat);
      softdebug.print(" ");
      softdebug.print(gps_aprs_lon);
      softdebug.print(" ");
      softdebug.print(gps_altitude);
      softdebug.print(" ");
      softdebug.print(gps_course);
      softdebug.print(" ");
      softdebug.print(gps_speed);
      softdebug.print(" ");
      softdebug.println(gps_fix_age);
      softdebug.println("exiting ublox_to_aprs()");    
     
  #endif
  
  
}



