/*
                  Pinout of Teensy 4.1 on Rover as of 03/27/2023
                              AVAILABLE PORTS: 17 
                                               
                                  |||||||||  
                                  | Micro |
                               GND|       |5V
                                 0|       |GND
                                 1|       |3.3V
           RC Receiver Steer --> 2|*     *|23 <-- Right Limit Switch
        RC Receiver Throttle --> 3|*     *|22 <-- Left Limit Switch
          RC Receiver Toggle --> 4|*     *|21 <-- Rear Limit Switch
                                 5|      *|20 
                                 6|      *|19 <-- Compass SCL0 - Wire.begin()
                                 7|      *|18 <-- Compass SDA0 - Wire.begin()
                  Turn Servo --> 8|*     *|17 <-- Left Ultrasonic ECHO
                         ESC --> 9|*     *|16 <-- Left Ultrasonic TRIG
    CS0 - Vision Sensor SPI --> 10|*     *|15 <-- GPS RX3 - Serial3.begin()
  MOSI0 - Vision Sensor SPI --> 11|*     *|14 <-- GPS TX3 - Serial3.begin()
  MISO0 - Vision Sensor SPI --> 12|*     *|13 <-- Vision Sensor SPI - SCK0
                              3.3V|       |GND
                                24|      *|41 <-- Right Ultrasonic TRIG
                                25|      *|40 <-- Right Ultrasonic ECHO
                                26|      *|39 <-- Center Ultrasonic TRIG - UNUSED
                                27|      *|38 <-- Center Ultrasonic ECHO - UNUSED
                                28|       |37 
                                29|       |36
                                30|      *|35 <-- Bluetooth RX8 - Serial8.begin()
                                31|      *|34 <-- Bluetooth TX8 - Serial8.begin()
                                32|       |33
                                  |SD Card|
                                  |||||||||

 */
