///////////////////////////////////////////////////////////////////  
/*                    2xAD9850 2-tone oscillator                 */
///////////////////////////////////////////////////////////////////
/*  MCU:              STM32F411 (ARM Cortex M4)                  */
/*  Hardware:         Black Pill Board                           */
/*  Compiler:         GCC (GNU ARM TOOLCHAIN)                    */
/*  Author:           Peter Baier  (DK7IH)                       */
/*  Last change:      NOV 2021                                   */
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

  ///////////////////////////
 //   2xAD9850 on PORT A  //
///////////////////////////
// PIN and PORT definitions for 
#define DDS_GPIO GPIOA
#define W_CLK   0   //PA0 blue
#define FQ_UD_0 1   //PA2 white
#define FQ_UD_1 2   //PA2 white
#define SDATA   3   //PA3 green
#define RES     4   //PA4 pink

  //////////////////
 //    LCD 162   //
//////////////////
#define LCD_GPIO GPIOA
#define LCD_RS  5 //PA5 green
#define LCD_RW  6 //PA6 blue
#define LCD_E   7 //PA7 yellow
#define LCD_D0  8 //PA8 violet
#define LCD_D1  9 //PA9 lightgreen
#define LCD_D2 10 //PA10 gray
#define LCD_D3 11 //PA11 orange
#define DATAPIN0 8   //Set to number of first data pin on MCU, e. g. set 
                     //to 8 if data port starts with A8 the value would be 8

  ///////////////////////////
 //   EEPROM 24C65        //
///////////////////////////
#define EEPROMSIZE 8192
uint8_t device_addr = 0xA0; //I2C address of 24C65 E0, E1, E2 to GND
void eeprom24c65_write(uint16_t, uint8_t);
uint8_t eeprom24c65_read(uint16_t);
void eeprom_set_freq(uint16_t memaddr, uint16_t f);
uint16_t eeprom_get_freq(uint16_t memaddr);

  /////////
 // I2C //
/////////
void i2c_start(void);
void i2c_stop(void); 
void i2c_write_1byte(uint8_t, uint8_t);
void i2c_write_nbytes(uint8_t*, uint8_t); 
int16_t  i2c_read(uint8_t); 

  //////////////
 //   DDS    //
//////////////
void set_frequency(unsigned long, int);
void spi_send_bit(int);

  //////////////
 //   LCD    //
//////////////
void lcd_write(char, unsigned char);
void lcd_write(char, unsigned char);
void lcd_init(void);
void lcd_cls(void);
void lcd_line_cls(int);
void lcd_putchar(int, int, unsigned char);
void lcd_putstring(int, int, char*);
int lcd_putnumber(int, int, long, int, int, char, char);
void lcd_display_test(void);
int lcd_check_busy(void);

  //////////////
 //   MISC   //
//////////////
static void delay_ms(unsigned int); 
static void delay_us(unsigned int); 
int16_t get_keys(void);

  /////////////////////////
 // Rotary encoder etc. //
/////////////////////////
int16_t rotate = 0;    //Detected direction
int16_t laststate = 0; //Last state of rotary encoder
//Seconds
uint32_t pulses = 0;

extern "C" void EXTI0_IRQHandler(void);
extern "C" void TIM2_IRQHandler(void);

  ////////////////////////////////
 // IRQ Handler Rotary encoder //
////////////////////////////////
extern "C" void EXTI0_IRQHandler(void) 
{ 
	uint16_t state; 
		
	if(EXTI->PR & (1 << 0))  //Check first if the interrupt is triggered by EXTI0 
	{ 
		state = GPIOB->IDR & 0x03; //Read pins
        if(state & 1)
        {
            if(state & 2)
            {
                //Turn CW
                rotate = 1;
            }
            else
            {
                //Turn CCW
                rotate = -1;
            }
        }
        //Clear pending bit
        EXTI->PR = (1 << 0);
    } 
}    

  /////////////////////////////
 //     TIM2 INT Handler    //
/////////////////////////////
extern "C" void TIM2_IRQHandler(void)//IRQ-Handler for TIM2
{	
    if (TIM2->SR & TIM_SR_UIF)       //Toggle LED on update event every 500ms
    {
		pulses = 0;
    }
    TIM2->SR = 0x0;                  //Reset status register  
}

  /////////////////////////////
 //  Cheap and dirty delay  //
/////////////////////////////
static void delay_ms(unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

static void delay_us(unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2; j++);
    }    
}

  /////////////////////////////
 //         I 2 C           //
/////////////////////////////
void i2c_start(void) 
{
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

void i2c_stop(void) 
{
    I2C1->CR1 |= I2C_CR1_STOP;
    while(!(I2C1->SR2 & I2C_SR2_BUSY));
}

void i2c_write_1byte(uint8_t regaddr, uint8_t data) 
{
    //Start signal
    i2c_start();

    //Send chipaddr to be targeted
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); //Wait until transfer done
    //Perform one dummy read to clear register flags etc.
    (void)I2C1->SR2; //Clear addr reg

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF)); //Wait until transfer done

    //Send data
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));  //Wait

    //Send stop signal
    i2c_stop();
}

//Write multiple number of bytes (>1)
void i2c_write_nbytes(uint8_t *data, uint8_t n) 
{
	int t1 = 0;
	    
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = device_addr; 
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;

    for(t1 = 0; t1 < n; t1++)
    {
		//Send data
        I2C1->DR = data[t1];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }    
    
    //Send stop signal
    i2c_stop();
}

int16_t i2c_read(uint8_t regaddr) 
{
    int16_t reg;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = device_addr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}


int16_t i2c_read2(uint16_t regaddr) 
{
    int16_t reg;
    int16_t r_msb, r_lsb;
    
    r_msb = (regaddr & 0xFF00) >> 8;
    r_lsb = regaddr & 0x00FF;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = device_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register MSB
    I2C1->DR = r_msb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send operation type/register LSB
    I2C1->DR = r_lsb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = device_addr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}

  //////////////
 //    SPI   // 
//////////////
void spi_send_bit(int sbit)
{
    if(sbit) //Set/reset bit
	{
		DDS_GPIO->ODR |= (1 << SDATA);  
	}
	else
	{
		DDS_GPIO->ODR &= ~(1 << SDATA);
	}
	//Send clock signal
	DDS_GPIO->ODR |= (1 << W_CLK);  
    DDS_GPIO->ODR &= ~(1 << W_CLK);   
}
//Set AD9850 to desired frequency
void set_frequency(unsigned long fx, int module)
{
    unsigned long clk = 125000000;
    double fword0;
    unsigned long  fword1;
    uint8_t t1;
    		
	//Define frequency word
    fword0 = (double) fx / clk * 4294967296;
    fword1 = (unsigned long) (fword0);
     
    //Send 32 frequency bits + 8 control & phase additional bits to DDS
	//Start sequence
	if(!module)
	{
		DDS_GPIO->ODR &= ~(1 << FQ_UD_0); //FQ_UD 0 lo
	}	
    else
    {
		DDS_GPIO->ODR &= ~(1 << FQ_UD_1); //FQ_UD 1 lo
	}	
       
	//W0...W31
	for(t1 = 0; t1 < 32; t1++)
    {
       spi_send_bit(fword1 & (1 << t1));
    }
	
	//W32...W39
	for(t1 = 0; t1 < 8; t1++)
	{
	    spi_send_bit(0);
	}	
	//Stop  sequence
	if(!module)
	{
		DDS_GPIO->ODR |= (1 << FQ_UD_0); //FQ_UD 0 lo   
	}	
    else
    {
		DDS_GPIO->ODR |= (1 << FQ_UD_1); //FQ_UD 1 lo   
	}	
}

  ///////////////
 //   L C D   //
///////////////
// Write CMD or DATA to LCD
void lcd_write(char lcdmode, unsigned char value)
{
    uint8_t nh = (value >> 4) & 0x0F;
    uint8_t nl = value & 0x0F;
    
    while(lcd_check_busy()); //Check busy flag
    	
	LCD_GPIO->ODR &= ~(1 << LCD_RW);   //Set RW to write operation, i. e. =0
	    
    if(!lcdmode)
	{
        LCD_GPIO->ODR &= ~(1 << LCD_RS); //CMD
	}	
    else
	{
        LCD_GPIO->ODR |= (1 << LCD_RS);   //DATA
	}	
	
	LCD_GPIO->ODR &= ~(0x0F << DATAPIN0); //Reset data port
	
	//HI NIBBLE        
    LCD_GPIO->ODR |= (1 << LCD_E);
    LCD_GPIO->ODR |= (nh << DATAPIN0);
    LCD_GPIO->ODR &= ~(1 << LCD_E);
	
	LCD_GPIO->ODR &= ~(0x0F << DATAPIN0); //Reset data port
		
	//LO NIBBLE        
	LCD_GPIO->ODR |= (1 << LCD_E);
    LCD_GPIO->ODR |= (nl << DATAPIN0);
    LCD_GPIO->ODR &= ~(1 << LCD_E);
}

int lcd_check_busy(void)
{
    int t1;
	unsigned char value;
	
	//LCD_PORT data bits (0:3) on rx mode
	for(t1 = 0; t1 < 4; t1++)
	{
	    GPIOE->MODER  &= ~(3 << (t1 << 1)); //Set to 00 -> Input mode
        GPIOE->PUPDR  &= ~(3 << (t1 << 1)); //Set to 00	-> No pullup nor pulldown
    } 
	
    LCD_GPIO->ODR &= ~(0x01);
    LCD_GPIO->ODR |= (1 << LCD_RW);  //Read operation => RW=1
	
	LCD_GPIO->ODR &= ~(1 << LCD_RS); //CMD => RS=0: for busy flag
	
	//Read data
	//Hi nibble
	LCD_GPIO->ODR |= (1 << LCD_E);          //E=1
    delay_us(100);       
	value = (LCD_GPIO->IDR & 0x0F) << 4;
    LCD_GPIO->ODR &= ~(1 << LCD_E);       //E=0	
		
	//Lo nibble
	LCD_GPIO->ODR |= (1 << LCD_E);          //E=1
    delay_us(100);       
	value += (LCD_GPIO->IDR & 0x0F);
    LCD_GPIO->ODR &= ~(1 << LCD_E);       //E=0	
		
	//Put pin D0..D3 in general purpose output mode
    LCD_GPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    	
	LCD_GPIO->ODR |= 0x01;   
	
	return (value >> 8) & 1;
}  

//Send one char to LCD
void lcd_putchar(int row, int col, unsigned char ch)
{
    lcd_write(0, col + 128 + row * 0x40);
    lcd_write(1, ch);
}

//Print out \0-terminated string on LCD
void lcd_putstring(int row, int col, char *s)
{
    unsigned char t1 = col;

    while(*(s))
	{
        lcd_putchar(row, t1++, *(s++));
	}	
}

//Clear LCD
void lcd_cls(void)
{
    lcd_write(0, 1);
}

//Init LCD
void lcd_init(void)
{
		   
    //Basic settings of LCD
    //4-Bit mode, 2 lines, 5x7 matrix
    lcd_write(0, 0x28);
    delay_ms(2);
    lcd_write(0, 0x28);
    delay_ms(2);
    
    //Display on, Cursor off, Blink off 
    lcd_write(0, 0x0C);
    delay_ms(2);

    //No display shift, no cursor move
    lcd_write(0, 0x04);
    delay_ms(2);
}

//Write an n-digit number (int or long) to LCD
int lcd_putnumber(int row, int col, long num, int digits, int dec, char orientation, char showplussign)
{
    char cl = col, minusflag = 0;
    unsigned char cdigit[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, digitcnt = 0;
    long t1, t2, n = num, r, x = 1;

    if(num < 0)
    {
        minusflag = 1;
        n *= -1;
    }

    /* Stellenzahl automatisch bestimmen */
    if(digits == -1)
    {
        for(t1 = 1; t1 < 10 && (n / x); t1++)
		{
            x *= 10;
		}	
        digits = t1 - 1;
    }

    if(!digits)
        digits = 1;

    for(t1 = digits - 1; t1 >= 0; t1--)
    {
        x = 1;
        for(t2 = 0; t2 < t1; t2++)
        {
            x *= 10;
        }    
        r = n / x;
        cdigit[digitcnt++] = r + 48;

        if(t1 == dec) 
        {
            cdigit[digitcnt++] = 46;
        }    
        n -= r * x;
    }

    digitcnt--;
    t1 = 0;

    /* Ausgabe */
    switch(orientation)
    {
        case 'l':   cl = col;
                    if(minusflag)
                    {
                        lcd_putchar(row, cl++, '-');
                        digitcnt++;
                    }	 
		            else
		            {
		                if(showplussign)
			            {
			                lcd_putchar(row, cl++, '+');
                            digitcnt++;
			            } 
                    }	
			
                    while(cl <= col + digitcnt)                       /* Linksbuendig */
		            {
                        lcd_putchar(row, cl++, cdigit[t1++]);
					}	
                    break;

        case 'r':   t1 = digitcnt;                              /* Rechtsbuendig */
                    for(cl = col; t1 >= 0; cl--)              
					{
                        lcd_putchar(row, cl, cdigit[t1--]);
                        if(minusflag)	
						{
                            lcd_putchar(row, --cl, '-');
                        }
					}	
    }
	
    if(dec == -1)
	{
        return digits;
	}	
    else
	{
        return digits + 1;	
	}	
}	

void lcd_line_cls(int ln)
{
    int t1; 
	
	for(t1 = 0; t1 < 15; t1++)
	{
	    lcd_putchar(ln, t1, 32);
	}
}	

//Define chars
void defcustomcharacters(void)
{
    int i1;
    unsigned char adr = 0x40;

    unsigned char customchar[]={0x0C, 0x10, 0x18, 0x12, 0x12, 0x02, 0x02, 0x02, //f1
	                            0x0C, 0x10, 0x18, 0x17, 0x11, 0x07, 0x04, 0x07, //f2
	                            0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,  
	                            0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 
	                            0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, //End S-Mater chars
	                            0x00, 0x00, 0x0E, 0x0E, 0x0E, 0x0E, 0x00, 0x00,
	                            0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00,
	                            0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E, 0x0E};
    lcd_write(0, 0);
    lcd_write(1, 0);

    //Send data to CGRAM in LCD
    for (i1 = 0; i1 < 64; i1++)
    {
        lcd_write(0, adr++);
        lcd_write(1, customchar[i1]);
    }
}

  ///////////////////////
 //   EEPROM 24C65    //
///////////////////////
void eeprom24c65_write(uint16_t mem_address, uint8_t value)
{
   uint8_t data[3]; 
   data[0] = mem_address >> 8;   //Address of byte in 24C65 MSB
   data[1] = mem_address & 0xFF; //                         LSB
   data[2] = value;
   
   i2c_write_nbytes(data, 3);
}	

uint8_t eeprom24c65_read(uint16_t mem_address)
{
   uint8_t r = i2c_read2(mem_address);
   delay_ms(5);
   
   return r;
}

uint16_t eeprom_get_freq(uint16_t memaddr)
{
	uint16_t f;
    
    f = eeprom24c65_read(memaddr * 2) << 8;
    delay_ms(10);
    f += eeprom24c65_read(memaddr * 2 + 1);
    
    return f;
}	

void eeprom_set_freq(uint16_t memaddr, uint16_t f)
{
     eeprom24c65_write(memaddr * 2, (f >> 8) & 0xFF);
     delay_ms(10);
     eeprom24c65_write((memaddr * 2 + 1), f & 0xFF);
     delay_ms(10);
}	

  //////////////
 //   KEYS   // 
//////////////
int16_t get_keys(void)
{
	//return GPIOB->IDR >> 12;
	
	switch(GPIOB->IDR >> 12)
	{
		case 14: return 0;
		         break;
		         
		case 13: return 1;
		         break;
		         
		case 11: return 2;
		         break;
	}
	return -1;
}			

  //////////////
 //   MAIN   // 
//////////////
int main(void)
{
	unsigned long fx;
	int16_t rotate_old = 0;
	uint16_t f[2] = {1000, 1200};
	int sel = 0;
	int t1;
	
	 //////////////////////////////////////////////
    // Set SystemClock to 100 MHz with 25 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;        //2 wait state for 96 MHz
    RCC->CR |= RCC_CR_HSEON;                    //Activate external clock (HSE: 8 MHz)
    while ((RCC->CR & RCC_CR_HSERDY) == 0);     //Wait until HSE is ready
    
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;     //PLL source is HSE
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;          //PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 20 << RCC_PLLCFGR_PLLM_Pos;  // -> f.VCO.in = 25MHz / 20 = 1.25MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
    RCC->PLLCFGR |= 200 << RCC_PLLCFGR_PLLN_Pos; //PLL-N: f.VCO.out = f.VCO.in * 200 = 250MHz
    
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;          //PLL-P: Main PLL (PLL) division factor for main system clock
                                                //f.PLL.output.clock = f.VCO.out / 2 = 100MHz
                                                
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;          //Main PLL (PLL) division factor for USB OTG FS, SDIO and 
                                                //random number generator clocks (f<=48MHz for RNG!, 48MHz for USB)
    RCC->PLLCFGR |= 4 << RCC_PLLCFGR_PLLQ_Pos;  //PLL-Q: f.VCO.out / 4 = 25MHz
        
    RCC->CR |= RCC_CR_PLLON;                    //Activate PLL (Output: 100 MHz)
    while ((RCC->CR & RCC_CR_PLLRDY) == 0);     //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= RCC_CFGR_HPRE_DIV2             //AHB divider:  /2
              | RCC_CFGR_PPRE1_DIV2             //APB1 divider: /2
              | RCC_CFGR_PPRE2_DIV2;            //APB2 divider: /2
               
    RCC->CFGR |= RCC_CFGR_SW_PLL;               //Switching to PLL clock source
    
	
	///////////////////////////////////
    //    Set up LEDs - GPIOC 13     //
    ///////////////////////////////////
    //Turn on the GPIOC peripheral for LED
    RCC->AHB1ENR |= (1 << 2);           //GPIOC power up
	GPIOC->MODER |= (1 << (13 << 1));	//Set PC13 for output

    //Turn on the GPIOA peripheral for DDS
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //RCC->AHB1ENR |= (1<<0);
    
    ///////////////////////////
    //Port Setup DDS and LCD //
    ///////////////////////////
    //DDS
    //Put pin A0..A3 in general purpose output mode
    DDS_GPIO->MODER  |=  (1 << (W_CLK << 1));	
    DDS_GPIO->MODER  |=  (1 << (FQ_UD_0 << 1));	
    DDS_GPIO->MODER  |=  (1 << (FQ_UD_1 << 1));	
    DDS_GPIO->MODER  |=  (1 << (SDATA << 1));	
    DDS_GPIO->MODER  |=  (1 << (RES << 1));	
    
    //LCD
    LCD_GPIO->MODER  |=  (1 << (LCD_RS << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_RW << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_E << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D0 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D1 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D2 << 1));	
    LCD_GPIO->MODER  |=  (1 << (LCD_D3 << 1));	
    
    /////////////////////////////////////
    // Setup I2C - GPIOB 6 SCK, 9 SDA  //
    /////////////////////////////////////
    //Enable I2C clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    //Setup I2C PB6 and PB9 pins
    RCC->AHB1ENR |= (1 << 1);         //GPIOB on
    GPIOB->MODER &= ~(3 << (6 << 1)); //PB6 as SCK
    GPIOB->MODER |=  (2 << (6 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 6);        //open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //open-drain

    //Choose AF4 for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     //for PB6 SCK
    GPIOB->AFR[1] |= (4 << (1 << 2));     //for PB9 SDA

    //Turn on the GPIOA peripheral for LED
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //RCC->AHB1ENR |= (1<<0);
    GPIOC->MODER |= (1 << (13 << 1));	
    GPIOC->ODR |= (1 << 13);
	 
    //Reset and clear register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    //Set I2C clock
    I2C1->CR2 |= (10 << 0); // 10Mhz periph clock
    I2C1->CCR |= (50 << 0);

    //Maximum rise time.
    I2C1->TRISE |= (11 << 0); //Set TRISE to 11 eq. 100khz
    
    I2C1->CR1 |= I2C_CR1_PE; //Enable i2c
    //I2C init procedure accomplished. //////////////////////////

    /////////////////////////
    //TIMER                //
    /////////////////////////
    //Enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
    
    //Timer calculation
    //Timer update frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1) 
    TIM2->PSC = 5000; //Dive system clock (f=50MHz) by 5000 -> update frequency = 10000/s
    TIM2->ARR = 500;    //Define overrun after 500ms

    //Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 2); //Priority level 2
    NVIC_EnableIRQ(TIM2_IRQn);      //Enable TIM2 IRQ from NVIC
    
    TIM2->CR1 |= (1 << 0);           //Enable Timer 2 module (CEN, bit0)
    
    /////////////////////////
    //Rotary Encoder Setup //
    /////////////////////////
    //Set PB0, PB1 as input pins
    RCC->AHB1ENR |= (1 << 1);                           //GPIOB power up
    GPIOB->MODER &= ~((3 << (0 << 1))|(3 << (1 << 1))); //PB0 und PB1 for Input
    GPIOB->PUPDR |= (1 << (0 << 1))|(1 << (1 << 1));    //Pullup PB0 und PB1
    
    RCC->APB2ENR |= (1 << 14); //Enable SYSCFG clock (APB2ENR: bit 14)
    
    SYSCFG->EXTICR[0] |= 0x0001;  //Write 0b01 to map PB0 to EXTI0
    EXTI->RTSR |= 0x01;           //Enable rising edge trigger on EXTI0
    EXTI->IMR |= 0x01;            //Mask EXTI0

    NVIC_SetPriority(EXTI0_IRQn, 1); //Set Priority for each interrupt request Priority level 1
    NVIC_EnableIRQ(EXTI0_IRQn);      //Enable EXT0 IRQ from NVIC

    ////////////////////
    //Input Key Setup //
    ////////////////////
    RCC->AHB1ENR |= (1 << 1);   //GPIOB power up
    for(t1 = 12; t1 < 15; t1++)
    {
        GPIOB->MODER  &= ~(3 << (t1 << 1)); //Set to 00 -> Input mode
        GPIOB->PUPDR  &= ~(3 << (t1 << 1)); //Reset to 00	-> No pullup nor pulldown
        GPIOB->PUPDR  |=  (1 << (t1 << 1)); //Set to 01 -> Pull up
    }
    
    ////////////////////
    //Init procedures //
    ////////////////////    
    //Reset AD9850
    DDS_GPIO->ODR |= (1 << RES);   //Bit set
    delay_ms(1);                   //wait for > 20ns i. e. 1ms minimum time with _delay_s()
	DDS_GPIO->ODR &= ~(1 << RES);  //Bit erase        
	
	//Display init
	delay_ms(200); 
    lcd_init();
    delay_ms(200); 
    lcd_cls();		
    delay_ms(200); 
    defcustomcharacters();		
        	
    lcd_putstring(0, 0, (char*)"2-Tone Osc.");
    lcd_putstring(1, 0, (char*)"by DK7IH");
    delay_ms(1000);
    lcd_line_cls(1);
    
    GPIOC->ODR |= (1 << 13); //LED off
    
    //Load frequencies
    for(t1 = 0; t1 < 2; t1++)
    {
		fx = eeprom_get_freq(t1);
		if((fx < 10000) && (fx > 100))
		{
			f[t1] = fx;
		}	
		set_frequency(f[t1], t1);
		set_frequency(f[t1], t1);
	}	
    
    lcd_putchar(1, 0, 0);
    lcd_putchar(1, 1, '=');
    lcd_putchar(1, 7, 1);
    lcd_putchar(1, 8, '=');
    lcd_putnumber(1, 2, f[0], -1, -1, 'l', 0);
    lcd_putstring(1, 14, (char*) "Hz");
    lcd_putnumber(1, 9, f[1], -1, -1, 'l', 0);
    lcd_putchar(0, 15, sel);
    
	for(;;)
	{
		if(rotate != rotate_old)
		{
			pulses++;
		    if(sel)
		    {
				if(rotate > 0)
				{
					if(f[1] < 9990)
					{
				        f[1] += pulses;
				        set_frequency(f[1], sel);
				    }   
				}
				
				if(rotate < 0)
				{
					if(f[1] > 10)
					{
				        f[1] -= pulses;
				        set_frequency(f[1], sel);
				    }    
				}
				lcd_putstring(1, 9, (char*)"     ");
				lcd_putnumber(1, 9, f[1], -1, -1, 'l', 0);
			}
			else
			{
				if(rotate > 0)
				{
					if(f[0] < 9990)
					{
				        f[0] += pulses;
				        set_frequency(f[0], sel);
				    }   
				}
				
				if(rotate < 0)
				{
					if(f[0] > 10)
					{
				        f[0] -= pulses;
				        set_frequency(f[0], sel);
				     }   
				}
				lcd_putchar(1, 0, 0);
				lcd_putstring(1, 2, (char*)"     ");
				lcd_putnumber(1, 2, f[0], -1, -1, 'l', 0);
			}	
		    rotate_old = rotate;
		    rotate = 0;
		}
		   
		switch(get_keys())
		{
			case 0: sel = 0;
			        lcd_putchar(0, 15, sel);
			        while(get_keys() != -1);
			        break;
			case 1: sel = 1;
			        lcd_putchar(0, 15, sel);
			        while(get_keys() != -1);
			        break; 
			case 2: eeprom_set_freq(0, f[0]);
			        eeprom_set_freq(1, f[1]); 
			        lcd_putstring(0, 12, (char*)"OK");
			        while(get_keys() != -1);
			        lcd_putstring(0, 12, (char*)"  ");
			        
		}	        
	}	
}
