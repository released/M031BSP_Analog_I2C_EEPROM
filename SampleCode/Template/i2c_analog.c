
/* Includes ------------------------------------------------------------------*/
#include "i2c_analog.h"
#include <stdio.h>
#include "NuMicro.h"

/*-------------------------------------------------------------------------------*/

uint32_t u32_I2C_SW_Delay;

uint8_t I2C_ANALOG_GPIO_DeInit(void)
{
	uint8_t i = 0;
	
	for (i = 0 ; i < 9 ; i++)	
	{
		SET_SCL; 
		I2C_ANALOG_DELAY;
		RESET_SCL;
		I2C_ANALOG_DELAY;
	}
	
	I2C_ANALOG_Stop();

	return I2C_ANALOG_BUS_READY;
}
void I2C_ANALOG_SW_open(uint32_t u32BusClock)
{
    if(u32BusClock > 480000)
        u32_I2C_SW_Delay = 1;
    else
        u32_I2C_SW_Delay = 480000 / u32BusClock; 


    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk )) |
                    (SYS_GPC_MFPL_PC0MFP_GPIO | SYS_GPC_MFPL_PC1MFP_GPIO);


    GPIO_SetMode(SCL_PORT, SCL_PIN_MASK, SCL_GPIO_MODE);
    GPIO_SetMode(SDA_PORT, SDA_PIN_MASK, SDA_GPIO_MODE);	

	SET_SDA;
	SET_SCL; 
}

void I2C_ANALOG_Delay(void)
{
	#if 1
	CLK_SysTickDelay(u32_I2C_SW_Delay);
	#else
	uint8_t i = 5;
	while (i)
	{
		i--;
		__asm("nop");
	}
	#endif
}

uint8_t I2C_ANALOG_Start(void)
{
    /* SCL high && SDA falling edge == I2C start signal */	
	
     SET_SDA;
     I2C_ANALOG_DELAY;

     SET_SCL;
     I2C_ANALOG_DELAY;

//     if( I2C_ANALOG_SDA_STATE == DISABLE )
//     {
//          return I2C_ANALOG_BUS_BUSY;
//     }

     RESET_SDA;
     I2C_ANALOG_DELAY;

     RESET_SCL;
     I2C_ANALOG_DELAY;

//     if( I2C_ANALOG_SDA_STATE == ENABLE )
//     {
//          return I2C_ANALOG_BUS_ERROR;
//     }

     return I2C_ANALOG_BUS_READY;
}

void I2C_ANALOG_Stop(void)
{
	/* SCL high && SDA raise edge == I2C stop signal */

     RESET_SDA;
     I2C_ANALOG_DELAY;
     SET_SCL;
     I2C_ANALOG_DELAY;
     SET_SDA;
     I2C_ANALOG_DELAY;
}


void I2C_ANALOG_SendNACK(void)
{
	#if 1
	SET_SDA;
	#else
	RESET_SDA;
	I2C_ANALOG_DELAY;
	#endif

	SET_SCL;
	I2C_ANALOG_DELAY;

	RESET_SCL;
	I2C_ANALOG_DELAY;
}

void I2C_ANALOG_SendACK(void)
{
	#if 1
	RESET_SDA;
	I2C_ANALOG_DELAY;
	#else
	SET_SDA;
	I2C_ANALOG_DELAY;
	#endif

	SET_SCL;
	I2C_ANALOG_DELAY;

	RESET_SCL;
	I2C_ANALOG_DELAY;

	SET_SDA;	
}

uint8_t I2C_ANALOG_WaitACK(void)
{
//    RESET_SCL;
//    I2C_ANALOG_DELAY;

    SET_SDA;
    I2C_ANALOG_DELAY;
    SET_SCL; 	 
    I2C_ANALOG_DELAY;
	
    if (I2C_ANALOG_SDA_STATE)
    {
        RESET_SCL;
        I2C_ANALOG_DELAY;
        return I2C_ANALOG_BUS_ERROR;
    }

    RESET_SCL;
    I2C_ANALOG_DELAY;
    
    return I2C_ANALOG_BUS_READY;
}

uint8_t I2C_ANALOG_SendByte(uint8_t Data)
{
     uint8_t i;
//     RESET_SCL;
     for(i=0;i<8;i++)
     { 
		if(Data & 0x80)
		{
		    SET_SDA;
		}
		else
		{
		    RESET_SDA;
		}

		I2C_ANALOG_DELAY;
		SET_SCL;
		I2C_ANALOG_DELAY;
		RESET_SCL;
		if ( i == 7 ) 
		{ 
			SET_SDA; 
		}

		Data <<= 1;		
		I2C_ANALOG_DELAY;		
     }

	#if 1
	return I2C_ANALOG_WaitACK();
	#else
//     SET_SDA;
//     I2C_ANALOG_DELAY;
//     SET_SCL;
//     I2C_ANALOG_DELAY;  
//     if(I2C_ANALOG_SDA_STATE)
//     {
//           RESET_SCL;
//           return I2C_ANALOG_NACK;
//     }
//     else
//     {
//           RESET_SCL;
//           return I2C_ANALOG_ACK; 
//     } 
	#endif
}

uint8_t I2C_ANALOG_RecvByte(void)
{
     uint8_t i,Dat = 0;
//     SET_SDA;
//     RESET_SCL;
     Dat = 0;
     for(i=0; i<8; i++)
     {
		Dat <<= 1;	 	
		SET_SCL;          
		I2C_ANALOG_DELAY;

		#if 1
		Dat += I2C_ANALOG_SDA_STATE;
		#else
		if(I2C_ANALOG_SDA_STATE)
		{
		Dat|=0x01;
		}  
		#endif
		RESET_SCL;         
		I2C_ANALOG_DELAY;               
     }
     return Dat;
}

uint8_t I2C_ANALOG_WriteData(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t* REG_data, uint16_t count)
{
	#if 1
	uint16_t i = 0;	
	I2C_ANALOG_Start();                  
	I2C_ANALOG_SendByte(SlaveAddress); 
	
	I2C_ANALOG_SendByte(REG_Address >> 8); 	
	I2C_ANALOG_SendByte(REG_Address & 0xFF);
    
	for (i = 0; i < count; i++) 
	{
		I2C_ANALOG_SendByte(REG_data[i]);
	}
	
	I2C_ANALOG_Stop(); 

    return I2C_ANALOG_BUS_READY;	
	#else	// single
	I2C_ANALOG_Start();                  
	I2C_ANALOG_SendByte(SlaveAddress);   
	I2C_ANALOG_SendByte(REG_Address);    
	I2C_ANALOG_SendByte(REG_data);       
	I2C_ANALOG_Stop(); 
	#endif
}


uint8_t I2C_ANALOG_ReadData(uint8_t SlaveAddress,uint16_t REG_Address,uint8_t* REG_data, uint16_t count)
{ 
	#if 1
	uint16_t i = 0;	
	I2C_ANALOG_Start();                          
	I2C_ANALOG_SendByte(SlaveAddress);        
	
	I2C_ANALOG_SendByte(REG_Address >> 8); 	
	I2C_ANALOG_SendByte(REG_Address & 0xFF);            

	I2C_ANALOG_Start();  
	I2C_ANALOG_SendByte(SlaveAddress | I2C_RD); 

	for (i = 0; i < count; i++) 
	{
		REG_data[i] = I2C_ANALOG_RecvByte();		
		if (i == (count - 1)) 
		{
			//Last byte
			I2C_ANALOG_SendNACK();			
		} 
		else 
		{
			I2C_ANALOG_SendACK(); 			
		}
	}
	I2C_ANALOG_Stop(); 
    return I2C_ANALOG_BUS_READY;
	
	#else	// single
	uint8_t REG_data;
	I2C_ANALOG_Start();                          
	I2C_ANALOG_SendByte(SlaveAddress);           
	I2C_ANALOG_SendByte(REG_Address);              
	I2C_ANALOG_Start();                          
	I2C_ANALOG_SendByte(SlaveAddress+1);         
	REG_data = I2C_ANALOG_RecvByte();            
	I2C_ANALOG_SendNACK();//I2C_ANALOG_SendACK();  
	I2C_ANALOG_Stop();                           
	return REG_data;
	#endif
}


uint8_t I2C_ANALOG_WriteMulti(uint8_t address, uint16_t reg, uint8_t* data, uint16_t count) 
{

    uint16_t i;
	
    if (I2C_ANALOG_Start()!=I2C_ANALOG_BUS_READY)
    {
        return I2C_ANALOG_BUS_ERROR;//I2C_ANALOG_GPIO_DeInit();
    }
//    I2C_ANALOG_SendByte(address);
    if (!I2C_ANALOG_SendByte(address))//(!I2C_ANALOG_WaitACK())
    {
        I2C_ANALOG_Stop();
        return I2C_ANALOG_BUS_ERROR;
    }
    
    I2C_ANALOG_SendByte(reg);
//    I2C_ANALOG_WaitACK();
    
    for (i = 0; i < count; i++) 
    {
//        I2C_ANALOG_SendByte(data[i]);
        if (!I2C_ANALOG_SendByte(address))//(!I2C_ANALOG_WaitACK()) 
		{
            I2C_ANALOG_Stop();
            return I2C_ANALOG_BUS_ERROR;//I2C_ANALOG_GPIO_DeInit();
        }
    }
    
    I2C_ANALOG_Stop();
    
    return I2C_ANALOG_BUS_READY; 
}

uint8_t I2C_ANALOG_ReadMulti(uint8_t address, uint16_t reg, uint8_t* data, uint16_t count) 
{

    if (I2C_ANALOG_Start()!=I2C_ANALOG_BUS_READY)
    {
        return I2C_ANALOG_BUS_ERROR;//I2C_ANALOG_GPIO_DeInit();
    }	
	
//    I2C_ANALOG_SendByte(address);
    if (!I2C_ANALOG_SendByte(address))//(!I2C_ANALOG_WaitACK())
    {
        I2C_ANALOG_Stop();
        return I2C_ANALOG_BUS_ERROR;//I2C_ANALOG_GPIO_DeInit();
    }
    I2C_ANALOG_SendByte(reg);
//    I2C_ANALOG_WaitACK();
    I2C_ANALOG_Start();
    I2C_ANALOG_SendByte(address | I2C_RD);
//    I2C_ANALOG_WaitACK();
    while (count) 
    {
        *data = I2C_ANALOG_RecvByte();
        if (count == 1)
        {
            I2C_ANALOG_SendNACK();
        }
        else
        {
            I2C_ANALOG_SendACK();
        }
        data++;
        count--;
    }
    I2C_ANALOG_Stop();
	
    return I2C_ANALOG_BUS_READY;
}



