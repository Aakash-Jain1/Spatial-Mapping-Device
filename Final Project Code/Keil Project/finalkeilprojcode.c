/* 
2DX3 Final Project Code
Author: Aakash Jain
Modified: April 4th, 2025
*/

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK 0x00000008    // Data Acknowledge Enable
#define I2C_MCS_DATACK 0x00000008 // Acknowledge Data
#define I2C_MCS_ADRACK 0x00000004 // Acknowledge Address
#define I2C_MCS_STOP 0x00000004   // Generate STOP
#define I2C_MCS_START 0x00000002  // Generate START
#define I2C_MCS_ERROR 0x00000002  // Error
#define I2C_MCS_RUN 0x00000001    // I2C Master Enable
#define I2C_MCS_BUSY 0x00000001   // I2C Busy
#define I2C_MCR_MFE 0x00000010    // I2C Master Function Enable

#define MAXRETRIES 5 // number of receive attempts before giving up

void I2C_Init(void)
{
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;   // activate I2C0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1; // activate port B
    while ((SYSCTL_PRGPIO_R & 0x0002) == 0) //wait for port B to be ready
    {
    }; 

    GPIO_PORTB_AFSEL_R |= 0x0C; // 3) enable alt funct on PB2,3  0b00001100 SCL
    GPIO_PORTB_ODR_R |= 0x08;   // 4) enable open drain on PB3 only    SDA

    GPIO_PORTB_DEN_R |= 0x0C; // 5) enable digital I/O on PB2,3
                              //    GPIO_PORTB_AMSEL_R &= ~0x0C;           // 7) disable analog functionality on PB2,3

    // 6) configure PB2,3 as I2C
    //  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200; // TED The register is bitwise-cleared for the relevant bits (0xFFFF00FF), 
																																			 //and then it is updated to select the I2C functions (0x00002200 for PB2 and PB3).
    I2C0_MCR_R = I2C_MCR_MFE;                                          // 9) master function enable
    //    I2C0_MTPR_R = 0x3B;                                         
		I2C0_MTPR_R = 0x06; // 8) configure for 100 kbps clock 10u = 2 * (1+TIMER_PRD) * 10 * 71.4ns --- MTPR is 6 (timer prescaler) this effects the SCL clock signal
		//standard mode
}

// The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void)
{
    // Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6; // activate clock for Port N
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0)
    {
    };                           // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;    // make PG0 in (HiZ)
    GPIO_PORTG_AFSEL_R &= ~0x01; // disable alt funct on PG0
    GPIO_PORTG_DEN_R |= 0x01;    // enable digital I/O on PG0
                                 // configure PG0 as GPIO
    GPIO_PORTG_AMSEL_R &= ~0x01; // disable analog functionality on PN0

    return;
}


void PortH_Init(void)
{ 

    // Use PortM pins (PH0-PH3) for output // Im going to change this to PH0-PH3
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; // activate clock for Port M // changed this from Port M to Port 7(H)
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0)
    {
    };                           // allow time for clock to stabilize, SYSCTL_PRGPIO_R- provides ready status for GPIO ports, indicates if clock is stable and ready to be used.
    GPIO_PORTH_DIR_R |= 0x0F;    // configure Port M pins (PM0-PM3) as output 0000 1111
    GPIO_PORTH_AFSEL_R &= ~0x0F; // disable alt funct on Port H pins (PH0-PH3)  which measn they will behave as GPIO
    GPIO_PORTH_DEN_R |= 0x0F;    // enable digital I/O on Port H pins (PH0-PH3)
                                 // configure Port H as GPIO
    GPIO_PORTH_AMSEL_R &= ~0x0F; // disable analog functionality on Port H pins (PM0-PM3)
    return;
}


// Intialized for on-board push buttons
void PortJ_Init(void)
{ 

    // Use PortM pins (PM0-PM3) for output // Im going to change this to PH0-PH3
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8; // activate clock for Port J
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0)
    {
    };                           // allow time for clock to stabilize
    GPIO_PORTJ_DIR_R |= 0xFC;    // configure Port J pins (PJ0-PJ1) as input 1111 1100
    GPIO_PORTJ_AFSEL_R &= ~0x03; // disable alt funct on Port J pins (PJ0-PJ1)  which measn they will behave as GPIO
    GPIO_PORTJ_DEN_R |= 0x03;    // enable digital I/O on Port H pins (PJ0-PJ1)
                                 // configure Port J as GPIO
    GPIO_PORTJ_AMSEL_R &= ~0x03; // disable analog functionality on Port J pins (PJ0-PJ1)

    // GPIO_PORTJ_DATA_R |= 0x03;
    //  settign as active low because it is set as 1 by default .
    GPIO_PORTJ_PUR_R |= 0x03;
    return;
}

// Implimented for buttons
void PortF_Init(void)
{ 

    // Use PortM pins (PM0-PM3) for output // Im going to change this to PH0-PH3
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; // activate clock for Port J
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5) == 0)
    {
    };                           // allow time for clock to stabilize
    GPIO_PORTF_DIR_R |= 0x11;    // configure Port J pins (PJ0-PJ1) as output 0001 0001
    GPIO_PORTF_AFSEL_R &= ~0x11; // disable alt funct on Port J pins (PJ0-PJ1)  which measn they will behave as GPIO
    GPIO_PORTF_DEN_R |= 0x11;    // enable digital I/O on Port H pins (PJ0-PJ1)
                                 // configure Port J as GPIO
    GPIO_PORTF_AMSEL_R &= ~0x11; // disable analog functionality on Port J pins (PH0-PH1)
    // GPIO_PORTF_DATA_R |= 0x10;
    return;
}

// Implimented for LEDs
void PortN_Init(void)
{ 

    // Use PortM pins (PM0-PM3) for output // Im going to change this to PH0-PH3
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12; // activate clock for Port J
    while ((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0)
    {
    };                           // allow time for clock to stabilize
    GPIO_PORTN_DIR_R |= 0x03;    // configure Port J pins (PJ0-PJ1) as output 0000 0011
    GPIO_PORTN_AFSEL_R &= ~0x03; // disable alt funct on Port J pins (PJ0-PJ1)  which measn they will behave as GPIO
    GPIO_PORTN_DEN_R |= 0x03;    // enable digital I/O on Port H pins (PJ0-PJ1)
                                 // configure Port J as GPIO
    GPIO_PORTN_AMSEL_R &= ~0x03; // disable analog functionality on Port J pins (PH0-PH1)
    // GPIO_PORTN_DATA_R &= ~0x03;
    return;
}

void PortL_Init(void){
//Use PortN LED onboard
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R10; // activate clock for Port M
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R10) == 0){}; // allow time for clock to stabilize
GPIO_PORTL_DIR_R |= 0x1F;         // make PM0 out 0001 1111
  GPIO_PORTL_AFSEL_R &= ~0x1F;     // disable alt funct on PM0
  GPIO_PORTL_DEN_R |= 0x1F;         // enable digital I/O on PM0

  GPIO_PORTL_AMSEL_R &= ~0x1F;     // disable analog functionality on PM0

GPIO_PORTL_DATA_R ^= 0x1F; //hello world!
SysTick_Wait10ms(10); //.1s delay
GPIO_PORTL_DATA_R ^= 0x1F;
return;
}

int duty = 128; // duty cycle 50% for bus frequency testing

void DutyCycle_Percent(int duty){ //Create square wave for AD2 Bus Verification
float percent;
int time =0;
percent = (duty*1000)/255;  // for our duty percentage is 0.5 as we get 501.9 and we scaled it to get a whole number
int intPercent = (int)percent;
GPIO_PORTL_DATA_R ^= 0b00010000;//Toggle_Bit();  // Turn ON
SysTick_Wait10us(intPercent);  // ON time

GPIO_PORTL_DATA_R ^= 0b00010000;//Toggle_Bit();  // Turn OFF
SysTick_Wait10us((1000 - intPercent));  // OFF time
}


// XSHUT. This pin is an active-low shutdown input;
//  the board pulls it up to VDD to enable the sensor by default.
//  Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void)
{
    GPIO_PORTG_DIR_R |= 0x01;        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110; // PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(100);
    GPIO_PORTG_DIR_R &= ~0x01; // make PG0 input (HiZ)
}


// DELCARATIONS

volatile uint8_t motorState = 0;     // 0 = not running , 1 = running button 0
volatile uint8_t data_collection = 1; // 0 is not getting data, : button 1
volatile uint8_t data_flag = 0; 
int steps = 16;         // 11.25 degree rotations
int total_steps = 2048; // 16 steps per 11.25deg * 32 for full rotation of 360 deg * 4 per "Step"
int steps_count = 0;    // keep track of the number of steps to reach 512
int direction = 1;


// Spin motor either direction
// Full step implementation, 2 phases at a time
void spin()
{
    uint32_t delay = 1; // Delay between steps (adjust as needed)

    for (int i = 0; i < steps; i++)
    {

        if (direction == 1)
        {                                   // CLOCKWISE
            GPIO_PORTH_DATA_R = 0b00001001; // Step 1
            steps_count++;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100; // Step 2
            steps_count++;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110; // Step 3
            steps_count++;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000011; // Step 4
            steps_count++;
            SysTick_Wait10ms(delay);
        }
        else
        {                                   // COUNTERCLOCKWISE
            GPIO_PORTH_DATA_R = 0b00000011; // Step 4
            steps_count++;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110; // Step 3
            steps_count++;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100; // Step 2
            steps_count++;
            SysTick_Wait10ms(delay);
            GPIO_PORTH_DATA_R = 0b00001001; // Step 1
            steps_count++;
            SysTick_Wait10ms(delay);
        }

        // Every 11.25 degrees
        if ((i + 1) % steps == 0)
        {
            data_flag = 1; // LED flash happens here after TOF distance is taken
        }
        else
        {
            data_flag = 0; // If it is not stepping then do not take TOF measurements
        }

        // After a full rotation 360 flip the direction of the motor to handle wiring
        if (steps_count >= total_steps)
        {
            direction *= -1; // Toggle direction (1 <-> -1)
            steps_count = 0; // Reset step counter
        }
				
					
    }
}

int start_stop_button()                          // Button 0 for motor on and off
{                                              // Active low logic button so there is pull-up resistors 
    return (GPIO_PORTJ_DATA_R & 0x01) ? 0 : 1; // this is 0 if no button press, and 1 if there is button press
}

int collecting_data() // Button 1 for collecting data, active low logic
{
    return (GPIO_PORTJ_DATA_R & 0x02) ? 0 : 1; // 0 if not collecting data || 1 if are collecting data from , default is collecting TOF measurements when start button pressed
}



//*********** MAIN Function ************************


uint16_t dev = 0x29; // address of the ToF sensor as an I2C slave peripheral
int status = 0;

int main(void)
{
		// byteData: used to store a single byte of data temporarily during I2C communication or sensor reading
		// sensorState: stores the state of the sensor, initialized to 0 (used to track the current sensor status or state)
		// myByteArray: an array of 10 bytes, initialized with all values set to 0xFF,  used to store data received from the sensor 
    uint8_t byteData, sensorState = 0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, i = 0;
    uint16_t wordData;
    uint16_t Distance;
    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum;
    uint8_t RangeStatus;
    uint8_t dataReady;
		
    // initialize
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortH_Init();
    PortJ_Init();
    PortN_Init();
    PortF_Init();	
		PortL_Init();			


    // Inital messages to see transmission
    int mynumber = 1;
    sprintf(printf_buffer, "2DX Final Code %d\r\n", mynumber);
    UART_printf(printf_buffer);
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9

    status = VL53L1_RdByte(dev, 0x010F, &byteData); // for model ID (0xEA)
    sprintf(printf_buffer, "(Model_ID)=0x%x, \r\n", byteData);
    UART_printf(printf_buffer);
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9
		
    status = VL53L1_RdByte(dev, 0x0110, &byteData); // for model type (0xcc)
    sprintf(printf_buffer, "(Module_Type)=0x%x, \r\n", byteData);
    UART_printf(printf_buffer);
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9
		
    status = VL53L1_RdWord(dev, 0x010F, &wordData); // for model type (0xcc)
    sprintf(printf_buffer, "(Module_Type)=0x%x, \r\n", wordData);
    UART_printf(printf_buffer);
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9

    /* Those basic I2C read functions can be used to check your own I2C functions */
    status = VL53L1X_GetSensorId(dev, &wordData);

    sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);
    UART_printf(printf_buffer);
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9
		
    // Wait for sensor TOF to boot and keep checking until it does
    while (sensorState == 0)
    {
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);

    }
    FlashAllLEDs(); //Flash all leds at startup for visual assurance
    UART_printf("ToF Chip Booted! Please Wait...\r\n"); // Print out booted message for TOF
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9

    status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

    /* 2 Initialize the sensor with the default setting  */
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status); //Check if the sensor was initialized sucessfully, print out sucessful or error
		FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9
		
		//already long mode by default but explicity setting to long
		status = VL53L1X_SetDistanceMode(dev, 2); // 2 is long mode so measurements up to 4m which is needed for hallway scan, short can do max 1.3m
		

		
		//POLLING METHOD, ACTIVELY WAIT FOR BUTTON PRESSES	

		
    while (1)
    {
			DutyCycle_Percent(duty);
			
			if (start_stop_button() == 1) // if button 0 is pressed: motor will run
        {
            SysTick_Wait10ms(3); // debounce for button press interpertation
            if (start_stop_button() == 1)    // check if the button is still pressed after the wait
            {
                // Call spin by the value of motorState
                motorState = !motorState; // Switching the motor on or off

                GPIO_PORTF_DATA_R = (motorState ? 0b00010000 : ~0b00010001); // PF4 for MOTOR ON, led debugging otherwise all leds off
                while (start_stop_button() == 1)
                {
                }; // we are waiting for button release here.
            }
        }
        else
            data_flag = 0; // do not collect data when the button is not pressed and the motor and TOF measurement is not turned on


        if (collecting_data() == 1) // button 2 is pressed turn on or off TOF readings but keep motor spinning (desgin choice)
        {
            SysTick_Wait10ms(3);     // debounce delay
            if (collecting_data() == 1) // check if stil collecting data or not

            {
                data_collection = !data_collection;                 // switch between collecting or not collecting data
                while (collecting_data() == 1)
                {
                } // waiting for the button to be released to process
            }
        }
					

        // When motorState = 1, motor is running
        if (motorState == 1)
        {
            spin(); // run the spin 
        }
        else
        {
            GPIO_PORTN_DATA_R = 0x00; //turn off led for data collection and measurement when motor OFF 
        }

        status = VL53L1X_StartRanging(dev); // 4 This function has to be called to enable the ranging  ,used to make distance measurement

        if (data_flag == 1 && direction == 1 && data_collection == 1) // we are checking if it is running and spiining clockwise and allowed to take measurements
        {

            status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
            status = VL53L1X_GetDistance(dev, &Distance); // Extacted value in Python to get distance for scan --- time taken/2 * speed of light
            status = VL53L1X_GetSignalRate(dev, &SignalRate);
            status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
            status = VL53L1X_GetSpadNb(dev, &SpadNum);


            FlashLED2(1); //Flash LED2 when the TOF reads a measurement - Design Spec PN0 for digit 9 MEASUREMENT STATUS

            status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/

            sprintf(printf_buffer, "%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, SignalRate, AmbientRate, SpadNum);

            UART_printf(printf_buffer);
						FlashLED1(1); //Flash LED D1 PN1 when data get transmitted through UART to pc - Design Spec PN1 for digit 9
            SysTick_Wait10ms(5); 
        }

        VL53L1X_StopRanging(dev);
    }
}