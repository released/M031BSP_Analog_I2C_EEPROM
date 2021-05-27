
/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include	"project_config.h"
#include "i2c_analog.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/
volatile uint32_t BitFlag = 0;
volatile uint32_t counter_tick = 0;

// I2C
#define EEPROM_SLAVE_ADDR    					(0xA0)

uint8_t buffer[256] = {0};
uint8_t u8SlaveAddr = EEPROM_SLAVE_ADDR >>1;


/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}

void compare_buffer(uint8_t *src, uint8_t *des, int nBytes)
{
    uint16_t i = 0;	
	
    for (i = 0; i < nBytes; i++)
    {
        if (src[i] != des[i])
        {
            printf("error idx : %4d : 0x%2X , 0x%2X\r\n", i , src[i],des[i]);
			set_flag(flag_error , ENABLE);
        }
    }

	if (!is_flag_set(flag_error))
	{
    	printf("%s finish \r\n" , __FUNCTION__);	
		set_flag(flag_error , DISABLE);
	}

}

void reset_buffer(void *dest, unsigned int val, unsigned int size)
{
    uint8_t *pu8Dest;
//    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;

	#if 1
	while (size-- > 0)
		*pu8Dest++ = val;
	#else
	memset(pu8Dest, val, size * (sizeof(pu8Dest[0]) ));
	#endif
	
}

void copy_buffer(void *dest, void *src, unsigned int size)
{
    uint8_t *pu8Src, *pu8Dest;
    unsigned int i;
    
    pu8Dest = (uint8_t *)dest;
    pu8Src  = (uint8_t *)src;


	#if 0
	  while (size--)
	    *pu8Dest++ = *pu8Src++;
	#else
    for (i = 0; i < size; i++)
        pu8Dest[i] = pu8Src[i];
	#endif
}

void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void  dump_buffer_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while (nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02X ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

void delay(uint16_t dly)
{
/*
	delay(100) : 14.84 us
	delay(200) : 29.37 us
	delay(300) : 43.97 us
	delay(400) : 58.5 us	
	delay(500) : 73.13 us	
	
	delay(1500) : 0.218 ms (218 us)
	delay(2000) : 0.291 ms (291 us)	
*/

	while( dly--);
}


void delay_ms(uint16_t ms)
{
	TIMER_Delay(TIMER0, 1000*ms);
}

void I2Cx_WriteMulti(uint8_t address,uint16_t reg,uint8_t *data,uint16_t len)
{	
	I2C_ANALOG_WriteData(address << 1 , reg , data , len);

}

void I2Cx_ReadMulti(uint8_t address,uint16_t reg,uint8_t *data,uint16_t len)
{ 
	I2C_ANALOG_ReadData(address << 1 , reg , data , len);

}

void I2Cx_Init(void)	//PC1 : SCL , PC0 : SDA
{
//	I2C_ANALOG_GPIO_Init();	
	I2C_ANALOG_SW_open(100000);
}

void EEPROM_clear(void)
{
	uint16_t i = 0;	
	uint8_t temp = 0xFF;
	
	
	printf("clear EEPROM ..\r\n");	

	for (i = 0 ; i < 256 ; i++)
	{
		I2Cx_WriteMulti(u8SlaveAddr , i , &temp , 1);
		CLK_SysTickDelay(5000);
		printf(".");
		if ((i+1)%64 ==0)
        {
            printf("\r\n");
        }		
	}

	printf("done\r\n");	

}

void EEPROM_dump(void)
{

	reset_buffer(buffer , 0x00 , 256);
	
	printf("dump EEPROM\r\n");

	#if 1
	I2Cx_ReadMulti(u8SlaveAddr , 0x00 , buffer , 256);
	dump_buffer_hex(buffer , 256);
	
	#else
	for (i = 0 ; i < 0x100 ; i++ )
	{	
		I2Cx_ReadMulti(u8SlaveAddr , 0x01 , &buffer[i] , 1);			
		printf("0x%2X," ,buffer[i]);

		if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }
	}	
	#endif
}

void EEPROM_test(void)
{
	uint8_t value = 0;
	uint16_t reg = 0;	
//	uint8_t array[2] = {0};
//	uint16_t i = 0;
	const uint32_t delay = 5000;

	#if 0	//clear EEPROM
	EEPROM_clear();
	#endif

	CLK_SysTickDelay(delay);
	reg = 0x01;
	value = 0xA1;
	I2Cx_WriteMulti(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(delay);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMulti(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);

	reg = 0x02;		
	value = 0xB2;
	I2Cx_WriteMulti(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(delay);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMulti(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);

	reg = 0x03;	
	value = 0xC3;		
	I2Cx_WriteMulti(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(delay);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMulti(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);

	reg = 0x04;		
	value = 0xD4;
	I2Cx_WriteMulti(u8SlaveAddr , reg , &value , 1);
	CLK_SysTickDelay(delay);
	printf("WR : 0x%2X : 0x%2X \r\n" ,reg ,value);

	value = 0;
	I2Cx_ReadMulti(u8SlaveAddr , reg , &value , 1);
	printf("RD : 0x%2X : 0x%2X \r\n" ,reg ,value);	

//	array[1] = 0x12;
//	array[0] = 0x46;	
//	reg = 0x1320;	
//	I2Cx_WriteMulti(u8SlaveAddr , reg , array , 2);
//	CLK_SysTickDelay(delay);
//	printf("WR : 0x%2X : 0x%2X , 0x%2X \r\n" ,reg ,array[0],array[1]);

//	value = 0;
//	I2Cx_ReadMulti(u8SlaveAddr , reg , &value , 1);
//	printf("RD : 0x%2X : 0x%2X \r\n" ,reg , value);
//	value = 0;	
//	I2Cx_ReadMulti(u8SlaveAddr , reg+1 , &value , 1);
//	printf("RD : 0x%2X : 0x%2X \r\n" ,reg+1 ,value);

	#if 0	//dump EEPROM
	EEPROM_dump();
	#endif
	
}

void EEPROM_process(void)
{
	uint16_t i = 0;
	uint8_t value = 0;
	static uint8_t addr = 0;
	static uint8_t temp = 0;
		
	const uint8_t data1[16] = 
	{
		0x23 , 0x16 , 0x80 , 0x49 , 0x56 , 0x30 , 0x17 , 0x22 ,
		0x33 , 0x46 , 0x55 , 0x27 , 0x39 , 0x48 , 0x57 , 0x60			
	};

	if (is_flag_set(flag_WriteAddr))		// fix vaule , to incr address
	{
		set_flag(flag_WriteAddr , DISABLE);
	
		value = 0x01;
		I2Cx_WriteMulti(u8SlaveAddr , addr , &value , 1);
		printf("WR : 0x%2X : 0x%2X \r\n" , addr++ , value);
	}

	if (is_flag_set(flag_WriteData))		// incr vaule , to fix address
	{
		set_flag(flag_WriteData , DISABLE);
	
		value = temp++;
		addr = 0x10;
		I2Cx_WriteMulti(u8SlaveAddr , addr , &value , 1);
		printf("WR : 0x%2X : 0x%2X \r\n" , addr++ , value);	
	}

	if (is_flag_set(flag_WriteData1))
	{
		set_flag(flag_WriteData1 , DISABLE);

		addr = 0x40;
		I2Cx_WriteMulti(u8SlaveAddr , addr ,(uint8_t*) data1 , 16 );

		for ( i = 0 ; i < 16; i++)
		{
			printf("WR : 0x%2X : 0x%2X \r\n" , addr , data1[i]);				
		}		
	}

	if (is_flag_set(flag_WriteData2))
	{
		set_flag(flag_WriteData2 , DISABLE);

		addr = 0x00;
		for ( i = 0 ; i < 0x100; i++)
		{
			value = i; 
			addr = i; 			
			I2Cx_WriteMulti(u8SlaveAddr , addr , &value , 1);
			CLK_SysTickDelay(5000);			
			printf("WR : 0x%2X : 0x%2X \r\n" , addr , i);

		}		
	}	

	if (is_flag_set(flag_Dump))
	{
		set_flag(flag_Dump , DISABLE);
		EEPROM_dump();
	}

	if (is_flag_set(flag_Erase))
	{
		set_flag(flag_Erase , DISABLE);
		EEPROM_clear();
	}	
}


void TMR1_IRQHandler(void)
{
//	static uint32_t LOG = 0;
	static uint16_t CNT = 0;
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);

		tick_counter();

		if (CNT++ >= 1000)
		{		
			CNT = 0;
//        	printf("%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;

		}

    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}


void UARTx_Process(void)
{
	uint8_t res = 0;
	
	res = UART_READ(UART0);

	printf("UARTx_Process = %c\r\n" ,res);

	if (res == 'x' || res == 'X')
	{
		NVIC_SystemReset();
	}

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
			case '1' :
				set_flag(flag_Dump , ENABLE);

				break;

			case '2': 
				set_flag(flag_WriteAddr , ENABLE);
			
				break;

			case '3': 
				set_flag(flag_WriteData , ENABLE);			
		
				break;			

			case '4': 
				set_flag(flag_WriteData1 , ENABLE);			
		
				break;	

			case '5': 
				set_flag(flag_WriteData2 , ENABLE);			
		
				break;	


			case '0' : 
				set_flag(flag_Erase , ENABLE);

				break;
		
			case 'Z':
			case 'z':				
				NVIC_SystemReset();
				break;				
		}
	}
}

void UART02_IRQHandler(void)
{	

    if(UART_GET_INT_FLAG(UART0, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART0) == 0)
        {
            UARTx_Process();
        }
    }

    if(UART0->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(UART0, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void UART0_Init(void)
{
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);

	/* Set UART receive time-out */
	UART_SetTimeoutCnt(UART0, 20);

	UART0->FIFO &= ~UART_FIFO_RFITL_4BYTES;
	UART0->FIFO |= UART_FIFO_RFITL_8BYTES;

	/* Enable UART Interrupt - */
	UART_ENABLE_INT(UART0, UART_INTEN_RDAIEN_Msk | UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk);
	
	NVIC_EnableIRQ(UART02_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
	
    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Waiting for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_PCLK0, CLK_CLKDIV0_UART0(1));
	
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M031 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{

    SYS_Init();

    UART0_Init();

    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);

	I2Cx_Init();

	TIMER1_Init();

	EEPROM_test();

    /* Got no where to go, just loop forever */
    while(1)
    {
		EEPROM_process();

    }
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
