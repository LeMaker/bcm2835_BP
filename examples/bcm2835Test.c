/************************************************************
  Copyright (C), 2014, lemaker.org.
  FileName:      bcm2835Test.c
  Author:         LeMaker Team
  Version :       1.0
  Date:           2015.01.12
  Description:  be used to test BCM2835 library
  Function List: 
  History:
      <author>    <time>     <version >   <desc>
  LeMaker Team    2015.01.12	  1.0		  create
  
  Compile: gcc bcm2835Test.c -lbcm2835 -o bcm2835Test 
***********************************************************/

#include <bcm2835.h>
#include <stdio.h>

#define ARRAYLEN 26 /*exclude BCMGPIO0 and BCMGPIO1*/
#define FAIL -1
/*pwm set*/
#define PWM_PIN RPI_BPLUS_GPIO_J8_12
#define PWM_CHANNEL 0
#define PWM_CYCLE 1024
#define PWM_DUTY 512

/*edge detect set*/
#define EDGE_PIN RPI_BPLUS_GPIO_J8_07 //Phy07
typedef enum
{
    LOW_MODE  = 1, 
	 HIGH_MODE = 2,
	 FALLING_MODE = 3,
	 RISING_MODE = 4
} egdeMode;

/*i2c set*/
#define PCF8574_ADDR 0x20

/*spi set*/
//CS0: RPI_BPLUS_GPIO_J8_24
//CS1: RPI_BPLUS_GPIO_J8_26
#define PIN_CS RPI_BPLUS_GPIO_J8_24

static unsigned char bplusGpio[ARRAYLEN] = 
{  
    RPI_BPLUS_GPIO_J8_03,
    RPI_BPLUS_GPIO_J8_05,
    RPI_BPLUS_GPIO_J8_07,
    RPI_BPLUS_GPIO_J8_08,
    RPI_BPLUS_GPIO_J8_10,
    RPI_BPLUS_GPIO_J8_11,
    RPI_BPLUS_GPIO_J8_12,
    RPI_BPLUS_GPIO_J8_13,
    RPI_BPLUS_GPIO_J8_15,
    RPI_BPLUS_GPIO_J8_16,
    RPI_BPLUS_GPIO_J8_18,
    RPI_BPLUS_GPIO_J8_19,
    RPI_BPLUS_GPIO_J8_21,
    RPI_BPLUS_GPIO_J8_22,
    RPI_BPLUS_GPIO_J8_23,
    RPI_BPLUS_GPIO_J8_24,
    RPI_BPLUS_GPIO_J8_26,
    RPI_BPLUS_GPIO_J8_29, 
    RPI_BPLUS_GPIO_J8_31, 
    RPI_BPLUS_GPIO_J8_32, 
    RPI_BPLUS_GPIO_J8_33, 
    RPI_BPLUS_GPIO_J8_35,
    RPI_BPLUS_GPIO_J8_36, 
    RPI_BPLUS_GPIO_J8_37,   
    RPI_BPLUS_GPIO_J8_38,
    RPI_BPLUS_GPIO_J8_40,
};

void edge_detect_test(uint8_t pin, uint8_t  mode)
{
	switch(mode)
	{
		case LOW_MODE:/*low level detect*/
			bcm2835_gpio_len(pin);
			printf("Start to detect the low level on the Phy07!\n");
			while(1)
			{
				if(bcm2835_gpio_eds(pin))
				{
					printf("Phy07:Low level has been detected !\n");
					break;
				}
			}
		break;
		case HIGH_MODE:/*high level detect*/
			bcm2835_gpio_hen(pin);
			printf("Start to detect the High level on the Phy07!\n");
			while(1)
			{
				if(bcm2835_gpio_eds(pin))
				{
					printf("Phy07:High level has been detected !\n");
					break;
				}
			}			
		break;
		case FALLING_MODE:/*falling detect*/
			bcm2835_gpio_fen(pin);
			printf("Start to detect the falling on the Phy07!\n");
			while(1)
			{
				if(bcm2835_gpio_eds(pin))
				{
					printf("Phy07: Falling has been detected !\n");
					break;
				}
			}
		break;
		case RISING_MODE:/*rising detect*/
			bcm2835_gpio_ren(pin);
			printf("Start to detect the rising on the Phy07!\n");
			while(1)
			{
				if(bcm2835_gpio_eds(pin))
				{
					printf("Phy07:Rising has been detected !\n");
					break;
				}
			}
		break;
	}
}

void delay_ms( unsigned int x)
{
    unsigned int  i,j;
    for(i=0;i<x;i++)
       for(j=0;j<100;j++);
}
void ee_write(unsigned int BufferOffset,unsigned char data)
{
   unsigned char temp[]={0x84,0xff};

	bcm2835_gpio_write(PIN_CS, LOW); /*must be called*/
	delay_ms(500);

   bcm2835_spi_writenb(temp,2); 

	delay_ms(50);
   bcm2835_spi_transfer((unsigned char)(BufferOffset>>8));
	delay_ms(50);
   bcm2835_spi_transfer((unsigned char)BufferOffset);
	delay_ms(50);
   bcm2835_spi_transfer(data);
	delay_ms(50);
	bcm2835_gpio_write(PIN_CS, HIGH);  /*must be called*/
	delay_ms(300);
}
unsigned char ee_read(unsigned int BufferOffset)
{
   unsigned char temp;
   unsigned char temp2[1]={0xff};
	
	bcm2835_gpio_write(PIN_CS, LOW);  /*must be called*/
   delay_ms(500);
   bcm2835_spi_transfer(0xD4);
	delay_ms(50);
   bcm2835_spi_transfer(0xff);
	delay_ms(50);
   bcm2835_spi_transfer((unsigned char)(BufferOffset>>8));
	delay_ms(50);
   bcm2835_spi_transfer((unsigned char)BufferOffset);
	delay_ms(50);
   bcm2835_spi_transfer(0xff);
	delay_ms(50);

    bcm2835_spi_transfern(temp2,1); 
   
    delay_ms(50);
    bcm2835_gpio_write(PIN_CS, HIGH);  /*must be called*/
    delay_ms(300);

    return temp2[0];
}

/*
*	Function: out_test
* Description: be used to test output related functions
*/
void out_test(void)
{
	int i;

	printf("--------------->Start Test Pin Output<---------------\n");
  	for(i=0; i < ARRAYLEN; i++)  //led off default
  	{
  		bcm2835_gpio_fsel(bplusGpio[i], BCM2835_GPIO_FSEL_OUTP); //output
		bcm2835_gpio_write(bplusGpio[i], LOW);
  	}	

	for(i=0; i< ARRAYLEN; i++)
	{
		bcm2835_gpio_write(bplusGpio[i], HIGH); //led on
		bcm2835_delay(1000);
		bcm2835_gpio_write(bplusGpio[i], LOW); //led off
		bcm2835_delay(1000);
	}
	printf("--------------->Test Over Of Pin Output<---------------\n");
}

/*
*	Function: pud_test
* Description: be used to test input pull-down and pull-up related functions
*/
void pud_test(void)
{
	uint8_t value;
	int i;
	
	printf("--------------->Start Test Pin PUD<---------------\n");
	for(i=0; i < ARRAYLEN; i++)
  	{
  		bcm2835_gpio_fsel(bplusGpio[i], BCM2835_GPIO_FSEL_INPT); //input
  	}
	for(i=0; i< ARRAYLEN; i++)
	{
		bcm2835_gpio_set_pud(bplusGpio[i], BCM2835_GPIO_PUD_UP);
		bcm2835_delay(500);
		value = bcm2835_gpio_lev(bplusGpio[i]);
		if(1 != value)
			printf("BCM pin%d:expect value %d and actual value %d\n", bplusGpio[i], 1, value);
		
		bcm2835_gpio_set_pud(bplusGpio[i], BCM2835_GPIO_PUD_DOWN);
		bcm2835_delay(500);
		value = bcm2835_gpio_lev(bplusGpio[i]);
		if(0 != value)
			printf("BCM pin%d:expect value %d and actual value %d\n", bplusGpio[i], 0, value);
	}	
	printf("--------------->Test Over Of Pin PUD<---------------\n");
	
}

/*
*	Function: pwm_test
* Description: be used to test PWM related functions
*/
void pwm_test(void)
{
	printf("--------------->Start Test PWM<--------------\n");
	bcm2835_gpio_fsel(PWM_PIN, BCM2835_GPIO_FSEL_ALT5); //set Phy12 to pwm function
	/*pwm clock = 24M/240*/
	bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_16);
	/*pwm channel mode: cycle mode OR pulse mode*/
   bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);
	/*the pwm cycle = pwm clock * PWM_CYCLE */
   bcm2835_pwm_set_range(PWM_CHANNEL, PWM_CYCLE);
	//enable pwm
	//active state: High level
	//enable clock for PWM1
	//set the pwm duty
	bcm2835_pwm_set_data(PWM_CHANNEL, PWM_DUTY);
	printf("--------------->Test Over Of PWM<--------------\n");
}

/*
*	Function: edge_test
* Description: be used to test edge detect related functions
*/
void edge_test(void)
{
	printf("--------------->Start Test dege detect<---------------\n");
	bcm2835_gpio_set_pud(EDGE_PIN, BCM2835_GPIO_PUD_UP); //enable pull-up
	bcm2835_gpio_fsel(EDGE_PIN, BCM2835_GPIO_FSEL_INPT);   //set input

	edge_detect_test(EDGE_PIN, LOW_MODE);
	edge_detect_test(EDGE_PIN, HIGH_MODE);
	edge_detect_test(EDGE_PIN, FALLING_MODE);
	edge_detect_test(EDGE_PIN, RISING_MODE);	
	printf("--------------->Test Over Of Edge Detect<---------------\n");
}

/*
*	Function: i2c_test
* Description: be used to test I2C related functions by using the PCF8574 module
*/
void i2c_test(void)
{
	uint8_t i2cWBuf[10] = {0x00};
	uint8_t i2cRBuf[10] = {0X00};
	printf("--------------->Test I2C With Pcf8574<--------------\n");
       i2cWBuf[0] = 0x40;
	bcm2835_i2c_begin();
	bcm2835_i2c_setSlaveAddress(PCF8574_ADDR); //set the slave address
       bcm2835_i2c_set_baudrate(400000);  //set the speed of the SDA 400kb/s
	bcm2835_i2c_write(i2cWBuf,1); 
	
	bcm2835_i2c_read(i2cRBuf, 1);
	if(i2cWBuf[0] == i2cRBuf[0])
		printf("I2C interface work well !...\n");
	else
		printf("I2C interface work bad!...\n");
	
	bcm2835_i2c_end();
	printf("==============Test Over Of I2C=================\n");		
}

/*
*	Function: spi_test
* Description: be used to test SPI related functions by using the AT450BXX module
*
*   host             slave
*   MISO  < - >   MISO
*   MOSI  < - >   MOSI
*   CLK    < - >   CLK
*   CE0/1 < - >   CS
*/
void spi_test(void)
{
	int num;
	unsigned char wBuf = 0x3f;
	unsigned char rBuf;
	printf("--------------->Test SPI With AT450BXX<--------------\n");
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);	  // The default
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0); 				  // The default
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); 
	//bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
	//bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);

	bcm2835_gpio_fsel(PIN_CS, BCM2835_GPIO_FSEL_OUTP);    /*set CS, and must be called*/

	printf("SPI write 0x%x\n", wBuf);
	ee_write(0,wBuf);
	bcm2835_delay(50);

	rBuf = ee_read(0);
	printf("SPI read 0x%x\n", rBuf);
	if(wBuf == rBuf)
	{
		printf("SPI interface work well !...\n");
	}
	else
	{
		printf("SPI interface work bad !...\n");
	}
	 bcm2835_spi_end();	
}

void uSage(void)
{
	printf("Usage: bcm2835Test [out | pud | edge | pwm | i2c| spi]\n");
	return;
}
int main(int argc, char *argv[])
{
	if(1 != bcm2835_init())
		return FAIL;

	if(argc != 2)
	{
		uSage();
		return FAIL;
	}
	if (0 == strcasecmp(argv[1], "out"))
	{
		out_test();
	}
	else if (0 == strcasecmp(argv[1], "pud"))
	{
		pud_test();
	}
	else if (0 == strcasecmp(argv[1], "edge"))
	{
		edge_test();
	}
	else if(0 == strcasecmp(argv[1], "pwm"))
	{
		pwm_test();
	}
	else if(0 == strcasecmp(argv[1], "i2c"))
	{
		i2c_test();
	}
	else if(0 == strcasecmp(argv[1], "spi"))
	{
		spi_test();
	}
	else
	{
		uSage();
		return FAIL;
	}
	
	bcm2835_close();
	return 0;
}
