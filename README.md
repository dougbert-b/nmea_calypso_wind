# nmea\_calypso\_wind

An Arduino/ESP32S3 program to read BLE data from a Calypso wireless wind meter and re-transmit it over NMEA2000.
It outputs both NMEA0183 and NMEA2000 apparent wind speed and angle, as well
as NMEA2000 battery level.

My hardware design includes a Xiao ESP32S3 board, a SN65VHD230
CAN Bus transceiver board, and a 12-to-5 volt voltage regulator board.

Also has a Wifi Access Point for OTA firmware updates.

Also there is a BLE server that imitates the Calypso wind meter, and relays
data from the real one.  This allows things like the Calypso phone app to work
while the wind meter is connected to this device.

