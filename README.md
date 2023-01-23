# multi-sensor-d1mini
Multi Sensor with D1 Mini for up to 5 ModBus SDM electricity meters, gas meters via magnetic switch, D0 interface for Easymeter

List of materials:
Arduino IDE - Einstellung: ESP8266 Boards =>NodeMCU 1.0 (ESP-12E Module) ... MMU -> 16 kB Cache + 48 IRAM
- D1 Mini ESP8266 WLAN Board Mikrokontroller (https://www.ebay.de/itm/255283312779)
- USB Kabel

Gaszähler
- Mini Magnetkontakt Magnetschalter Reed  (https://www.ebay.de/itm/313796479195)

SDM Modbus Zähler
- TTL RS-485 Konverter Modul Max485 Adapter Bus Modbus (https://www.ebay.de/itm/255283194259)

D0 EasyMeter Zähler
- Fototransistor Fotodiode Photodiode 3mm rund schwarz 30V 30° 940nm 75mW (https://www.ebay.de/itm/334295002197)
- 10k Ohm Widerstand


/*  WEMOS D1 Mini
                     ______________________________
                    |   L T L T L T L T L T L T    |                 5V
                    |                              |                 |
                 RST|                             1|TX HSer          |
                  A0|                             3|RX HSer  ----------- Anschluss IR-------------  GND
                  D0|16                           5|D1 ----  R0  (RS-485)                 
                  D5|14                           4|D2 ----  DI  (RS-485)                
                  D6|12                    10kPUP_0|D3 ----  RE+DE (RS-485)             
   Zähler 2 ----  D7|13                LED_10kPUP_2|D4 ----  Magnetschalter----GND     
   Zähler 3 ----  D8|15                            |GND                                  
                 3V3|__                            |5V    
                       |                           |
                       |___________________________|
