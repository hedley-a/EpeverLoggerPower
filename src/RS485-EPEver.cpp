/*
 *    Epever RS485 datalogger by Aidan Hedley
 *    https://github.com/hedley-a/ESP_DataLogger
 *    Adapted from:-RS485 TO  WIFI ADAPTOR CODE
 *    https://github.com/chickey/RS485-WiFi-EPEver
 *    by Colin Hickey 2021
 * 
 *    THIS code proof of wake on reset > logdata > saveto RTC memory > sleep
 *    29/12/23 - not reading MODBUS bugs??
 * 
 *    Hardware Changes:
 *    To Add: Microchip 12F683 to time wake (reset) from sleep and reset on external input
 *    Logging: will be saving homeopathic amounts of data, 4 bytes hourly  - 8 bytes daily,  
 *    Just use EEPROM? space for 512 days of daily data. Write endurance 10,000 days
 *    RTC memory enough for last two days worth of hourly data..maybe pop some into EEPROM
 *    Software Changes:
 *    WiFi-GUI as AP - no LAN so direct GUI connection to mobile device - implemented 20/12/23
 *    Powersave: ESP runs once to log load and solar power: reset every 40 seconds by 12F683
 *    Epever RTC used hourly to save watt/hour figures to FRAM
 *    If load state toggled - ESP+wifi+GUI powered for 10 mins (use Epever RTC to time this?)
 *    Managed by GUI banner with timeout - reset by GUI pushbutton?
 *    Midday Load OFF - every day - 
 *    
 *    Most of  my work so far has consisted of deleting code.... 
 *   
 *    
 *    Version 0.00

*/

#include <Arduino.h>
//#include <ESPUI.h>
#include <ModbusMaster.h>
#include <string.h>


////////////////
#define DEBUG
//#define GUI_DEBUG
//#define MQTT_DEBUG
//#define INFLUX_DEBUG
////////////////

bool switch_load = true;
bool loadState = false;

//#include "settings.h"
#include "config.h"
//#include "gui.h"

ModbusMaster node;   // instantiate ModbusMaster object

#ifndef TRANSMIT_PERIOD
  #define TRANSMIT_PERIOD 30000
#endif
unsigned long time_now = 0;

void preTransmission()
{
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1); // tx enabled
}

void postTransmission()
{
  digitalWrite(MAX485_RE, 0); // rx enabled
  digitalWrite(MAX485_DE, 0);
}


void waitForSerial(unsigned long timeout_millis) {
  unsigned long start = millis();
  while (!Serial) {
    if (millis() - start > timeout_millis)
      break;
  }
}

void waitForSerial1(unsigned long timeout_millis) {
  unsigned long start = millis();
  while (!Serial1) {
    if (millis() - start > timeout_millis)
      break;
  }
}

void niceDelay(unsigned long delayTime)
{  
  unsigned long startTime = millis();
  while (millis() - startTime < delayTime)
  {
    yield();
  }
}

uint16_t ReadRegister(uint16_t Register) {
  // Read register at the address passed in

  niceDelay(50);
  node.clearResponseBuffer();
  uint8_t result = node.readInputRegisters(Register, 1);
  if (result == node.ku8MBSuccess)  {
    
    uint16_t Register = node.getResponseBuffer(0);

  } else  {
#ifdef DEBUG
    Serial1.print(F("Miss read - ")); 
    Serial1.print(Register);
    Serial1.print(F(", ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }
  return result;
}

void ReadValues() {  
  // clear old data
  //
  memset(rtc.buf,0,sizeof(rtc.buf));
  memset(live.buf,0,sizeof(live.buf));
  memset(stats.buf,0,sizeof(stats.buf));

  // Read registers for clock
  //
  niceDelay(50);
  node.clearResponseBuffer();
  uint8_t result = node.readHoldingRegisters(RTC_CLOCK, RTC_CLOCK_CNT);
  if (result == node.ku8MBSuccess)  {

    rtc.buf[0]  = node.getResponseBuffer(0);
    rtc.buf[1]  = node.getResponseBuffer(1);
    rtc.buf[2]  = node.getResponseBuffer(2);

  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read rtc-data, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif

  } 
  if (result==226)     ErrorCounter++;
  
  // read LIVE-Data
  // 
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(LIVE_DATA, LIVE_DATA_CNT);

  if (result == node.ku8MBSuccess)  {

    for(uint8_t i=0; i< LIVE_DATA_CNT ;i++) live.buf[i] = node.getResponseBuffer(i);
       
  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read liva-data, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }

  // Statistical Data
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(STATISTICS, STATISTICS_CNT);

  if (result == node.ku8MBSuccess)  {
    
    for(uint8_t i=0; i< STATISTICS_CNT ;i++)  stats.buf[i] = node.getResponseBuffer(i);
    
  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read statistics, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  } 

  /* 
  Mis-Read and  don't need!
  // BATTERY_TYPE
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(BATTERY_TYPE, 1);
  if (result == node.ku8MBSuccess)  {
    
    BatteryType = node.getResponseBuffer(0);

  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read BATTERY_TYPE, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }
  

  // EQ_CHARGE_VOLT
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(EQ_CHARGE_VOLT, 1);
  if (result == node.ku8MBSuccess)  {
    
    EQChargeVoltValue = node.getResponseBuffer(0);

  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read EQ_CHARGE_VOLT, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif

  }

  // CHARGING_LIMIT_VOLT
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(CHARGING_LIMIT_VOLT, 1);
  if (result == node.ku8MBSuccess)  {
    
    ChargeLimitVolt = node.getResponseBuffer(0);

  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read CHARGING_LIMIT_VOLT, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }

  // Capacity
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(BATTERY_CAPACITY, 1);
  if (result == node.ku8MBSuccess)  {
    
    BatteryCapactity = node.getResponseBuffer(0);

  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read BATTERY_CAPACITY, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  } 
  */
  // Battery SOC
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(BATTERY_SOC, 1);
  if (result == node.ku8MBSuccess)  {
    
    batterySOC = node.getResponseBuffer(0);
    
  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read batterySOC, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }

  // Battery Net Current = Icharge - Iload
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(  BATTERY_CURRENT_L, 2);
  if (result == node.ku8MBSuccess)  {
    
    batteryCurrent = node.getResponseBuffer(0);
    batteryCurrent |= node.getResponseBuffer(1) << 16;
    
  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read batteryCurrent, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }
 
  //if (!switch_load) {
    // State of the Load Switch
    niceDelay(50);
    node.clearResponseBuffer();
    result = node.readCoils(  LOAD_STATE, 1 );
    if (result == node.ku8MBSuccess)  {
      
      loadState = node.getResponseBuffer(0);
          
    } else  {
#ifdef DEBUG
      Serial1.print(F("Miss read loadState, ret val:"));
      Serial1.println(result, HEX);
      Serial1.flush();
 #endif
    }
  //}
  /* Mis-Read and don't Need
  // Read Model
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(CCMODEL, 1);
  if (result == node.ku8MBSuccess)  {
    
    CCModel = node.getResponseBuffer(0);
    
  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read Model, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }
  */
  // Read Status Flags
  niceDelay(50);
  node.clearResponseBuffer();
  result = node.readInputRegisters(  0x3200, 2 );
  if (result == node.ku8MBSuccess)  {

    uint16_t temp = node.getResponseBuffer(0);
#ifdef DEBUG 
    Serial1.print(F("Batt Flags : "));
    Serial1.println(temp);
    Serial1.flush();
#endif
    
    status_batt.volt = temp & 0b1111;
    status_batt.temp = (temp  >> 4 ) & 0b1111;
    status_batt.resistance = (temp  >>  8 ) & 0b1;
    status_batt.rated_volt = (temp  >> 15 ) & 0b1;
    
    temp = node.getResponseBuffer(1);
#ifdef DEBUG 
    Serial1.print(F("Chrg Flags : "));
    Serial1.println(temp, HEX);
    Serial1.flush(); 
#endif

    charger_mode = ( temp & 0b0000000000001100 ) >> 2 ;
#ifdef DEBUG 
    Serial1.print(F("charger_mode  : "));
    Serial1.println( charger_mode );
    Serial1.flush();
#endif
  } else {
#ifdef DEBUG
    Serial1.print(F("Miss read ChargeState, ret val:"));
    Serial1.println(result, HEX);
    Serial1.flush();
#endif
  }
}

void debug_output(){
#ifdef DEBUG
  //Output values to serial
  Serial1.printf("\n\nTime:  20%02d-%02d-%02d   %02d:%02d:%02d   \n",  rtc.r.y , rtc.r.M , rtc.r.d , rtc.r.h , rtc.r.m , rtc.r.s  );
  Serial1.print(  F("\r\nLive-Data:           Volt        Amp       Watt  "));
  Serial1.printf( "\r\n  Panel:            %7.3f    %7.3f    %7.3f ",  live.l.pV/100.f ,  live.l.pI/100.f ,  live.l.pP/100.0f );
  Serial1.printf( "\r\n  Batt:             %7.3f    %7.3f    %7.3f ",  live.l.bV/100.f ,  live.l.bI/100.f ,  live.l.bP/100.0f );
  Serial1.printf( "\r\n  Load:             %7.3f    %7.3f    %7.3f \n",  live.l.lV/100.f ,  live.l.lI/100.f ,  live.l.lP/100.0f );
  Serial1.printf( "\r\n  Battery Current:  %7.3f  A ",      batteryCurrent/100.f  );
  Serial1.printf( "\r\n  Battery SOC:      %7.0f  %% ",     batterySOC/1.0f  );
  Serial1.printf( "\r\n  Load Switch:          %s   ",     (loadState==1?" On":"Off") );
  /*
  Serial1.print(  F("\r\n\nStatistics:  "));
  Serial1.printf( "\r\n  Panel:       min: %7.3f   max: %7.3f   V", stats.s.pVmin/100.f  , stats.s.pVmax/100.f  );
  Serial1.printf( "\r\n  Battery:     min: %7.3f   max: %7.3f   V\n", stats.s.bVmin /100.f , stats.s.bVmax/100.f);

  Serial1.printf( "\r\n  Consumed:    day: %7.3f   mon: %7.3f   year: %7.3f  total: %7.3f   kWh",
      stats.s.consEnerDay/100.f  ,stats.s.consEnerMon/100.f  ,stats.s.consEnerYear/100.f  ,stats.s.consEnerTotal/100.f   );
  Serial1.printf( "\r\n  Generated:   day: %7.3f   mon: %7.3f   year: %7.3f  total: %7.3f   kWh",
      stats.s.genEnerDay/100.f   ,stats.s.genEnerMon/100.f   ,stats.s.genEnerYear/100.f   ,stats.s.genEnerTotal/100.f  );
  Serial1.printf( "\r\n  CO2-Reduction:    %7.3f  t\n",      stats.s.c02Reduction/100.f  );
  */
  Serial1.print(  F("\r\nStatus:"));
  Serial1.printf( "\r\n    batt.volt:         %s   ",     batt_volt_status[status_batt.volt] );
  Serial1.printf( "\r\n    batt.temp:         %s   ",     batt_temp_status[status_batt.temp] );
  Serial1.printf( "\r\n    charger.charging:  %s   \n\n",     charger_charging_status[ charger_mode] );
#endif
}


//---------------------------------------------------------------------------------------
void setup(void) {
  niceDelay(50);
  // modbus callbacks
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial1.begin(115200);
  Serial1.setTimeout(2000);
  while(!Serial1) { } // Wait for serial to initialize.
  Serial1.printf("Serial Init." );

  Serial.begin(115200);
  Serial.setTimeout(2000);
  while(!Serial) { } // Wait for serial to initialize.
 
  // init modbus in receive mode
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  niceDelay(50);
  postTransmission();

  // EPEver Device ID and Baud Rate
  node.begin(1, Serial);

  //----CODE FROM LOOP TO TAKE DATA ONCE

  String buffer;
  buffer.reserve(64);
  // Read Values from Charge Controller
  ReadValues();
  niceDelay(50);
  debug_output();
  Serial1.flush();
  //#ifdef DEBUG
    
  //#endif

  // code to detect load user change of state
  // Flag to RTC RAM and reset -next boot in loop with wi-fi and GUI
  // 40 seconds delay...


  // Configure LED pin as an output
  pinMode(LED, OUTPUT);

  // Turn the LED on (HIGH is the voltage level)
  digitalWrite(LED, HIGH);

  // Wait for 1000 milliseconds
  delay(1000);

  // Turn the LED off by making the voltage LOW
  digitalWrite(LED, LOW);
  //WiFi.disconnect( true ); //ensures the ESP enters deep sleep correctly
  delay( 1 ); //ensures the ESP enters deep sleep correctly
  Serial1.println("\r Going into deep sleep mode for 20 seconds");

  //ESP.deepSleep(5e6, WAKE_RF_DISABLED );// use WAKE_RF_DISABLED to keep WiFi off when waking up
  // Deep sleep mode for 5 seconds, the ESP8266 wakes up by itself when GPIO 16 (D0 in NodeMCU board) is connected to the RESET pin
  ESP.deepSleep(20e6, WAKE_RF_DISABLED );
  delay(100); //added after deepSleep to ensure the ESP goes to sleep properly



}

void loop() {
}