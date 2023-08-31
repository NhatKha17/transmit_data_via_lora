# Lora_Sx1278-DHT11-LCD11602602
 -Project include knowledges :
    1)DHT11 - Temperature and Humidity
    2)Display LCD16X02 I2C interface
    3)2 Lora SX1278 SPI interface
    4)ESP 8266 send data THINGSPEAK sever
    5)UART connect STM32F411 with ESP8266

Steps to take 
* STM32F103 :
  -Power 5v DC
  -Read Data DHT11 
  -Lora1 transmit data to STM32F411
*STM32F411:
  -Power 5v DC
  -Lora2 receive data from STM32F103 via Lora1
  -Process data
  -Display data for LCD1602
  -And send data to Thingspeak sever by ESP8266
    
