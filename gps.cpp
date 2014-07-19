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
unsigned int gps_fix_age = 9999;

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
  
 int year;
byte month, day, hour, minute, second, hundredths;
unsigned long fix_age;
  ublox.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
  sprintf(gps_time, "%02d%02d%02d", hour,minute,second);
  strcat(gps_time,'\0');

  // returns +/- latitude/longitude in degrees
  ublox.f_get_position(&gps_lat, &gps_lon, &fix_age);
 
  //convert gps_lat into aprs compatible char array
  if (gps_lat >= 0){ //positive latitude = northern hemisphere "0000.00N"
    int lat_deg=(int)(gps_lat);   //degrees in integer format
    float temp_lat_min=(((gps_lat-lat_deg)*60));
    int lat_min=(int)temp_lat_min;    //minutes in integer format
    int lat_dec_min=(int)(((temp_lat_min-lat_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lat, "%02d%02d.%02dN", lat_deg,lat_min,lat_dec_min);
    gps_aprs_lat[8]='\0';
  }
  else{   //negative latitude = southern hemisphere "0000.00N"
    int lat_deg=(int)(-gps_lat);   //degrees in integer format
    float temp_lat_min=(((-gps_lat-lat_deg)*60));
    int lat_min=(int)temp_lat_min;    //minutes in integer format
    int lat_dec_min=(int)(((temp_lat_min-lat_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lat, "%02d%02d.%02dS", lat_deg,lat_min,lat_dec_min);
    gps_aprs_lat[8]='\0';
  }
  
  //convert gps_lon into aprs compatible char array
  if (gps_lon <= 0){ //negative longitude = western hemisphere "00000.00W"
    int lon_deg=(int)(-gps_lon);   //degrees in integer format
    float temp_lon_min=(((-gps_lon-lon_deg)*60));
    int lon_min=(int)temp_lon_min;    //minutes in integer format
    int lon_dec_min=(int)(((temp_lon_min-lon_min)*100)+0.5);  //decimal minutes in integer format
        
    sprintf(gps_aprs_lon, "%03d%02d.%02dW", lon_deg,lon_min,lon_dec_min);
    gps_aprs_lon[9]='\0';
  }
  else{   //positive longitude = eastern hemisphere "00000.00E"
    int lon_deg=(int)(gps_lon);   //degrees in integer format
    float temp_lon_min=(((gps_lon-lon_deg)*60));
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
    softdebug.println();  
    softdebug.println("GPS Data: Time Lat Long APRS_Lat APRS_Long Alt Crs Spd LVP");  //Crs=course, Spd=speed,LVP=last valid position
    softdebug.print(gps_time);
    softdebug.print(" ");
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
  #endif

}


void update_fix_age(){
  
  unsigned long fix_age;
  long lat, lon;
  
  ublox.get_position(&lat, &lon, &fix_age);  // Last Valid Position in milliseconds
  gps_fix_age=(unsigned int)fix_age/1000;   // Last Valid Position in seconds

}


