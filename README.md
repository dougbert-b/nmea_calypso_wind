# nmea\_calypso\_wind

An Arduino/ESP32 program to read BLE data from a Calypso wireless wind meter and re-transmit it over NMEA2000.

Outputs both NMEA0183 heading and NMEA2000 heading and attitude. My hardware design also includes a SN65VHD230
CAN Bus transceiver board and a 12-to-5 volt voltage regulator board.

