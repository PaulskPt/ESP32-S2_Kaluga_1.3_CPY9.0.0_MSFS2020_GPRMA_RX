
## MSFS2020 GPS data receiver

Circutipython V9.0.0-alpha.2 example to use an Espressif ESP32-S2 Kaluga-1.3.

This is an ongoing project to display certain GPS position, groundspeed, direction of flight (and eventually: altitude) information,
in this case to the display of the Espressif Kaluga-1.

At startup, the script reads the contents of the files settings.toml, gprma2.csv and GPSvariants.csv.

The script contains a State Class, that contains a lot of variables, lists, dictionaries, that initially were global variables.

There are still a lot of global variables that have to do with the display. In future updates we assume to have less global variables.

The script is now set up to receive, decode and display GPS messages of the type GPRMA. It can also receive, decode and display GPS messages
of the type GPRMC and GPGGA, with some modifications of the script.

Since this script is created from a copy of another similar project, still some cleaning up of code and variables have to be done.
As the script is now, it is running flawlessly more than two hourse, receiving, decoding and displaying GPS data received from Microsoft
Flightsimulator 2020. The add-on "FSUIPC7" is used to send the GPS data via a COM-port (see "globCOMprt":"com17" in settings.toml) 
of the PC to the Adafruit CP2102 USB-to-RS232 converter.

The built-in NEOPIXEL led will blink three times in color green when a GPS message has been received, checked and is ready to be displayed.

## Hardware

Beside the Espressif ESP32-S2 Kaluga-1.3, to convert USB-to-RS232 serial data an Adafruit CP2102N Friend
- USB-UART CP2102N converter - USB-C socket - Adafruit 5335 board is used.

## Debug info
The State Class has the attribute my_deug. If this boolean variable is set to True, the script will print a lot of
info to the REPL screen.

## ToDo

The intention is to add WiFi connection and retrieve datetime stamps from an NTP server to update the internal RTC.
WiFi communication is already prepared in the current script.


