#include <MKL25Z4.H>
#include "uart.h"
#include "gps.h"
#include "vector.h"
#include "MKL25z_I2C.h"

#include <cmath>
#include <stdlib.h>
#include <string.h>

#define LED_NUM     3                   /* Number of user LEDs                */
#define ARRAY_LENGTH 20

#define BMP180_ADDRESS          0x77<<1 // I2C address of BMP180, eight bit address on mbed
#define BMP180_WHO_AM_I         0xD0    // WHO_AM_I id of BMP180, should return 0x55
#define BMP180_RESET            0xE0
#define BMP180_CONTROL          0xF4
#define BMP180_OUT_MSB          0xF6
#define BMP180_OUT_LSB          0xF7
#define BMP180_OUT_XLSB         0xF8


float average(float array[]);
float stddev(float array[]);
float first_average(float array[]);
float last_average(float array[]);
int falldetected(void);

volatile uint32_t msTicks;                            /* counts 1ms timeTicks */

float x_values[ARRAY_LENGTH] = {0};
float y_values[ARRAY_LENGTH] = {0};
float z_values[ARRAY_LENGTH] = {0};
float sigma[ARRAY_LENGTH] = {0};
float theta_z[ARRAY_LENGTH] = {0};



const float threshold_sig                = 1.0105;
const float threshold_theta_z            = 1.2639;
const float threshold_delta_theta_z      = 0.8398;
const float threshold_sigma_stddev_first = 0.2050;
const float threshold_xyz_variances_norm = 0.5905;


/*----------------------------------------------------------------------------
	Fall Detection functions
 *----------------------------------------------------------------------------*/

float average(float array[]){
    int i;
    float average;
    float sum = 0.0;
    for (i = 0; i<ARRAY_LENGTH; i++) sum += array[i];
    average = sum/((float)(ARRAY_LENGTH/2));
    return average;
}

float first_average(float array[]){
    int i;
    float average;
    float sum = 0.0;
    for (i = 0; i<ARRAY_LENGTH/2; i++) sum += array[i];
    average = sum/((float)(ARRAY_LENGTH/2));
    return average;
}
	
float last_average(float array[]) {
    int i;
    float average;
    float sum = 0.0;
    for (i = ARRAY_LENGTH/2; i<ARRAY_LENGTH; i++) sum += array[i];
    average = sum/((float)(ARRAY_LENGTH/2));
    return average;
}


float stddev(float array[]) {
    float stddev = 0;
    int i;
    float average_a = average(array);
    for(i = 0; i <ARRAY_LENGTH; i++) stddev += pow(array[i]-average_a,2);
    stddev = sqrt(stddev/(float)ARRAY_LENGTH);
    return stddev;
}

float first_stddev(float array[]) {
    float stddev = 0;
    int i;
    float average = first_average(array);
    for(i = 0; i <ARRAY_LENGTH/2; i++) stddev += pow(array[i]-average,2);
    stddev = sqrt(stddev/((float)ARRAY_LENGTH * 2));
    return stddev;
}

float last_stddev(float array[]) {
    float stddev = 0;
    int i;
    float average = first_average(array);
    for(i = 0; i <ARRAY_LENGTH/2; i++) stddev += pow(array[i]-average,2);
    stddev = sqrt(stddev/((float)ARRAY_LENGTH * 2));
    return stddev;
}

void READ_adxl(void) {
		float x, y, z, root;
		int i = 0;

		//I2C ACCELEROMETER ADRESS 0x1D
		const int adxl_read = (0x1D << 1) & 0xFE;
		const int adxl_write = (0x1D << 1) | 0x01;	

		char cmd[4];
		char read_buff_x[6] = {0};

		I2C_configure();	

		//Activate peripheral
		cmd[0]= 0x2A;			//Active register adress
		cmd[1]= 0x01;			//Write 1 to activation register
		I2C_send(adxl_write, cmd, 2, 0);	

		//Get xyz acceleration values from register
		cmd[0] = 0x01;				
		I2C_send(adxl_write, cmd, 1, 0);
		I2C_read(adxl_read, read_buff_x, 6, 1); // Read in 6 bytes, 2 per axis

		//Convert to x,y,z reading using msb and lsb, assume last two digits noise
		x = ((int)(read_buff_x[0] << 6) | (int)(read_buff_x[1] >> 2)) / 4096.0;
		y = ((int)(read_buff_x[2] << 6) | (int)(read_buff_x[3] >> 2)) / 4096.0;
		z = ((int)(read_buff_x[4] << 6) | (int)(read_buff_x[5] >> 2)) / 4096.0;
			 
		I2C_powerDown();	
		
		root = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
		
		for (i = (ARRAY_LENGTH - 1); i>0; i--){
				x_values[i] = x_values[i-1];
				y_values[i] = y_values[i-1];		
				z_values[i] = z_values[i-1];
				sigma[i] = sigma[i-1];
				theta_z[i] = theta_z[i-1];
			}
			
		sigma[0] = root;
		theta_z[0] = acos(z / root);
		x_values[0] = fabsf(x);
		y_values[0] = fabsf(y);
		z_values[0] = fabsf(z);
		
}

int falldetected(void){
		int fall = 0;
		char buffer[200];
		int length;
		int i;
	
		float sigma_stddev_ratio, theta_z_average_ratio, theta_z_average_delta, sigma_stddev_first, xyz_variances_norm;
		
		sigma_stddev_ratio = first_stddev(sigma) / last_stddev(sigma); 
		theta_z_average_ratio = first_average(theta_z) / last_average(theta_z);
		theta_z_average_delta = first_average(theta_z)  - last_average(theta_z);
		sigma_stddev_first = first_stddev(sigma);
		xyz_variances_norm = sqrt(pow(first_stddev(x_values),2) + pow(first_stddev(y_values), 2) + pow(first_stddev(z_values), 2));
		
		if (
			(sigma_stddev_ratio > threshold_sig || theta_z_average_ratio > threshold_theta_z)
			&&
		(theta_z_average_ratio > threshold_theta_z || theta_z_average_delta > threshold_delta_theta_z)
			&&
		(sigma_stddev_first > threshold_sigma_stddev_first || xyz_variances_norm > threshold_xyz_variances_norm)
		) {
		//FALL DETECTED
		fall = 1;
			
		length = sprintf (buffer, "Fall");
		for(i=0;i<length;i++)
		{
			uart0_putchar(buffer[i]);
		}
	}
		else {
					length = sprintf (buffer, "No \n");
		for(i=0;i<length;i++)
		{
			uart0_putchar(buffer[i]);
		}
			
		}
			
			
	return fall;	
}


/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void) {
  msTicks++;                        /* increment counter necessary in Delay() */
}

/*------------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *------------------------------------------------------------------------------*/
__INLINE static void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

/*------------------------------------------------------------------------------
  configer LED pins
 *------------------------------------------------------------------------------*/
__INLINE static void LED_Config(void) {

  SIM->SCGC5    |= (1UL <<  10) | (1UL <<  12);      /* Enable Clock to Port B & D */ 
  PORTB->PCR[18] = (1UL <<  8);                      /* Pin PTB18 is GPIO */
  PORTB->PCR[19] = (1UL <<  8);                      /* Pin PTB19 is GPIO */
  PORTD->PCR[1]  = (1UL <<  8);                      /* Pin PTD1  is GPIO */

	FPTB->PDOR |=  (1UL << 18);							// enable PTB18 as an output 
	FPTB->PDDR |= (1ul << 19);							//enable PTB19 as an output
}

/*------------------------------------------------------------------------------
  Switch on LEDs/pins
 *------------------------------------------------------------------------------*/
__INLINE static void LED_On (void) {
	
	FPTB->PDOR	|= (1UL << 18);													//Set Port B pin 18 high
	FPTB->PDOR	|= (1UL << 19);													//Set Port B pin 19 high
}

/*------------------------------------------------------------------------------
  Switch off LEDs
 *------------------------------------------------------------------------------*/
__INLINE static void LED_Off (void) {
	
	FPTB->PDOR	&= ~(1UL << 18);												//Set Port B pin 18 Low
	FPTB->PDOR	&= ~(1UL << 19);												//Set Port B pin 18 High
	
}
/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/

int main (void) {
	int fallDetect;	
	char buffer[200];

  SystemCoreClockUpdate();                      /* Get Core Clock Frequency */
  SysTick_Config(SystemCoreClock/1000);         /* Generate interrupt each 1 ms    */
		
  LED_Config();   
	uart0_init (41940, 9600);											//Initalize uart0 at 9600 baud rate to communicate with zbee module
	
	// UArt_0 initilization	
	SIM->SCGC5    |= (1UL <<  9) | (1UL <<  13);    //Uart_0 clock and Clock to the port (e.g. B/E(13))
	SIM->SCGC5    |= (1UL <<  13);									//Uart_0 clock speed
  SIM->SCGC4		|= (1UL << 10); 									//Enable Uart_0
	
	PORTE->PCR[20] = (0x4 << 8);		//Set UART0 pins to alternative 4 TX
 	PORTE->PCR[21] = (0x4 << 8);		//Set UART0 pins to alteratnative 4 RX
	
	//I2C initiliziation
	
	SIM->SCGC5    |= (1UL <<  10);				//Enable clock for I2C register on port B
	SIM->SCGC4    |= (1UL <<  6);					//Enable I2C Clock
	
	PORTB->PCR[0] = (0x2 <<  8); 					//set adxl scl	set port PTB0 as alternative 2
	PORTB->PCR[1] = (0x2 <<  8); 					//set adxl sda	set port PTB1 as alternative 2
 
  while(1) {
		READ_adxl();
		fallDetect = falldetected();
    LED_On ();
		Delay(30);
  }
}

