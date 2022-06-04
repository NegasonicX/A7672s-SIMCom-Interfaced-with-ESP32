# A7672s SIMCom Interfaced with ESP32
The given code covers all the basic needs to communicate with A7672s SIMCom Module with ESP32 to post data on MQTT

# GPIO Functions:
I am using ESP32 Devkit V1

* GPIO 2  : Builtin LED ( Devkit V1 )
* GPIO 17 : UART TX
* GPIO 16 : UART RX
* GPIO 21 : ENABLE of A7672s

# Understanding the Flow:
* This code is developed for ESP32 on Embedded C Language using FreeRTOS.
* Entire Communication is based on UART protocol between ESP and A7672s SIMCom.
* The default baud of SIMCom is 115200 with 8 data bits, None parity and 1 stop bit.
* There are several "AT" Commands over which SIMCom understands, performance and responses tasks.
* The code initializes AT Commands, followed by getting the IP of simcard used.
* Further, through SNTP we get the date-time for specified location.
* Lastly, MQTT is started with specified broker and payload is posted on "4GBOARD_SIMCOMM".
* We also subscribe to a topic "4GBOARD_SIMCOMM/cmd" to receive payload.
* **Note :**
* **1) Simcard used need to have whitelisting of SNTP or broker being used ( if simcard is not general one ).**
* **2) If SIMCom loses given MQTT Connection, then it would disconnect and stop MQTT by executing those AT Commands before restarting.**

# Conclusion:
* There is nothing to conclude this time, it's super cool and works flawlessly.
* I have used basic AT Commands to perform few task but there's a big list of things one can discover and do!
* I hope you all like it :-D
  
# Reference Material:
* https://microchip.ua/simcom/LTE/A76xx/A7682/A76XX%20Series_AT_Command_Manual_V1.06.pdf
* https://manuals.plus/m/d70efb93e7bade7e6e61c78447a99e33b2f6607d08d3fa59b8ac5c65f8bb201d.pdf
* https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/module/sim7680/A76XX%20Series_UART_Application%20Note_V1.02.pdf
