# M031BSP_Analog_I2C_EEPROM
 M031BSP_Analog_I2C_EEPROM

update @ 2021/05/27

1. use GPIO to emulate I2C (PC0 : SDA , PC1 : SCL) , to access EEPROM

2. use terminal (digit 0 ~ 5 ) to read / write EEPROM

digit 1 : dump EEPROM 

digit 2 : fix address , modify value

digit 3 : modify address , fix value

digit 4 : write 16 bytes data

digit 5 : write 256 bytes data

digit 0 : reset EEPROM to 0xFF

3. below is eeprom test screen capture ,

power on log message  , 

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/log_power_on.jpg)

power on LA capture , 

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/LA_power_on.jpg)

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/LA_WR.jpg)

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/LA_RD.jpg)

when press digit 4 , 

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/log_digit4_16bytes_write.jpg)

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/LA_WR_16bytes_digit4.jpg)

when press digit 1 , 

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/log_digit1_dump_eeprom.jpg)

![image](https://github.com/released/M031BSP_Analog_I2C_EEPROM/blob/main/LA_RD_multi_digit1.jpg)






