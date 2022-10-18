/*
 * SPI_Slave.c
 *
 * Created: 22/05/2022 15:37:13
 * Author : Criss
 */ 

#define F_CPU 16000000UL			/* Define CPU Frequency e.g. here its 8MHz */
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */
#include <stdio.h>			/* Include Std. i/p o/p file */
#include <string.h>			/* Include string header file */

#define DDR_SPI DDRB
#define PORT_SPI PORTB
#define SS PB2			/* Define SPI bus pins */
#define MOSI PB3			
#define MISO PB4
#define SCK PB5


#define DDR_LCD  DDRD			/* Define LCD data port direction */
#define PORT_LCD PORTD			/* Define LCD data port */
#define RS PD0				/* Define Register Select pin */
#define EN PD1 				/* Define Enable signal pin */

void SPI_Init();								/* SPI Initialize function */
char SPI_Transmit(char);						/* SPI transmit data function */
char SPI_Receive();								/* SPI Receive data function */

void LCD_Init (void);
void LCD_Command(unsigned char);
void LCD_Clear(void);
void LCD_Char(unsigned char);
void LCD_String (char *);
void LCD_gotoxy (char, char);

void delayms(unsigned int);
void delayus(unsigned int);

void setup()
{
	LCD_Init();
	SPI_Init();
	LCD_gotoxy(0,0);
	LCD_String("Slave Device");
	LCD_gotoxy(0,1);
	LCD_String("Receive Data:    ");
}

void loop()
{
	uint8_t count;
	char buffer[5];
	while (1)
	{
		count = SPI_Receive();
		sprintf(buffer, "%d   ", count);
		LCD_gotoxy(13,1);
		LCD_String(buffer);
	}
}

int main(void)
{
	setup();
	loop();
}

void SPI_Init()									/* SPI Initialize function */
{
	DDR_SPI &= ~((1<<MOSI)|(1<<SCK)|(1<<SS));		/* Make MOSI, SCK, SS pin direction as input pins */
	DDR_SPI |= (1<<MISO);							/* Make MISO pin as output pin */
	SPCR = (1<<SPE);							/* Enable SPI in slave mode */
}

char SPI_Transmit(char data)					/* SPI transmit data function */
{
	SPDR = data;								/* Write data to SPI data register */
	while(!(SPSR & (1<<SPIF)));					/* Wait till transmission complete */
	return(SPDR);								/* return received data */
}

char SPI_Receive()								/* SPI Receive data function */
{
	while(!(SPSR & (1<<SPIF)));					/* Wait till reception complete */
	return(SPDR);								/* return received data */
}

void LCD_Init(void)			/* LCD Initialize function */
{
	DDR_LCD = 0xFF;			/* Make LCD port direction as o/p */
	delayms(20);			/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x02);		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);      /* 2 line, 5*7 matrix in 4-bit mode */
	LCD_Command(0x0C);      /* Display on cursor off*/
	LCD_Command(0x06);      /* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);      /* Clear display screen*/
	delayms(2);
}

void LCD_Clear(void)
{
	LCD_Command(0x01);		/* Clear display */
	delayms(2);
	LCD_Command(0x80);		/* Cursor at home position */
}

void LCD_Command(unsigned char cmnd)
{
	PORT_LCD = (PORT_LCD & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	PORT_LCD &= ~ (1<<RS);		/* RS=0, command reg. */
	PORT_LCD |= (1<<EN);		/* Enable pulse */
	delayus(1);
	PORT_LCD &= ~ (1<<EN);

	delayus(200);

	PORT_LCD = (PORT_LCD & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	PORT_LCD |= (1<<EN);
	delayus(1);
	PORT_LCD &= ~ (1<<EN);
	delayms(3);
}

void LCD_Char(unsigned char data)
{
	PORT_LCD = (PORT_LCD & 0x0F) | (data & 0xF0); /* sending upper nibble */
	PORT_LCD |= (1<<RS);		/* RS=1, data reg. */
	PORT_LCD|= (1<<EN);
	delayus(1);
	PORT_LCD &= ~ (1<<EN);

	delayus(200);

	PORT_LCD = (PORT_LCD & 0x0F) | (data << 4); /* sending lower nibble */
	PORT_LCD |= (1<<EN);
	delayus(1);
	PORT_LCD &= ~ (1<<EN);
	delayms(2);
}

void LCD_String (char *str)		/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)		/* Send each char of string till the NULL */
	{
		LCD_Char(str[i]);
	}
}

void LCD_gotoxy (char pos, char row)	/* Send string to LCD with xy position */
{
	if(row == 0 && pos<16){
		LCD_Command((pos & 0x0F)|0x80);	/* Command of first line and required position<16 */
	}
	else if(row == 1 && pos<16){
		LCD_Command((pos & 0x0F)|0xC0);	/* Command of second line and required position<16 */
	}
}

void delayms(unsigned int ms) {
	while(ms--) {
		_delay_ms(1);
	}
}
void delayus(unsigned int us) {
	while(us--) {
		_delay_us(1);
	}
}