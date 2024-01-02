# ESP8266 based datalogger for EPEver Solar CC
- Thanks to https://github.com/chickey/RS485-WiFi-EPEver


# Features:
Hardware Changes:
 *    To Add: Microchip 12F683 to time wake (reset) from sleep and reset on external input
 *    Logging: will be saving homeopathic amounts of data, 4 bytes hourly  - 8 bytes daily,  
 *    Just use EEPROM? space for 512 days of daily data. Write endurance 10,000 days
 *    RTC memory enough for last two days worth of hourly data..maybe pop some into EEPROM
 *    Software Changes:
 *    WiFi-GUI as AP - no LAN so direct GUI connection to mobile device - implemented 20/12/23
 *    Powersave: ESP sleep > log load and solar power >sleep timer reset every 40 seconds
 *    Save watt/hour figures to FRAM - hourly and daily summary
 *    If load state toggled - ESP+wifi+GUI powered for 10 mins
 *    Adapt GUI 
 *    Midday Load OFF - every day - 
-
