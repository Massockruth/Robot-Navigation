/* EEL 4742L : Lab 10
* Names: Keila Souriac & Ruth Massock
* Spring 25
* Section: 0001
* Date: 04/8/2025
* Description: This code is for the final project, which uses the QTRK line sensor to sense were the RSLK robot is on a line,
*  and it displays a specific color depending on its position on the line, and the robot is able to move along the line,
*  by turning the robot in the correct direction based on the amount of sensors it senses.
* TA Code: 17441554201
* Time: 0:51
************************************************************************************************************
*/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

/* Global Variables */
typedef enum MotorState{Forward, Reverse, off} MotorState;
MotorState leftMotor,rightMotor;
//typedef enum SpeedState{Fast, Slow, Off} SpeedState;
typedef enum sensorCondition{ ALL_LEFT, MORE_LEFT, MIDDLE, MORE_RIGHT, ONLYRIGHT, OFF, STANDBY} sensorCondition;
typedef enum buttonStates{buttonON, buttonOFF} buttonStates;
typedef enum led2{ LED2OFF, RED, GREEN, BLUE, YELLOW, CYAN, WHITE} led2;
typedef enum led1{ LED1OFF, ONEHZ, TWOHZ, FOURHZ, ON, LED1MAINTAIN} led1;
buttonStates b0ButtonState, bnButtonState;
led1 LED1State = 0;
led2 LED2State = 0;
sensorCondition sensorCon = OFF;
uint8_t  sensorNum = 0, controlFlag = 0, lost =0; // triggers LED2 based off of number of toggles ofLED1
uint8_t leftcount, rightcount;
uint16_t DUTYL = 30;
uint16_t DUTYR = 30;
//uint16_t DUTY = 30;  //30
uint16_t PERIOD = 10; //10
volatile uint8_t BMP0;
//#define PERIOD 70 // //10  2000
#define CLOCKDIVIDER TIMER_A_CLOCKSOURCE_DIVIDER_48
#define LEFTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_4
#define RIGHTCHANNEL TIMER_A_CAPTURECOMPARE_REGISTER_3
#define PORT7PINS GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7

Timer_A_PWMConfig timerPWMConfig;

/* Function Prototypes */
void config432IO();
void configRobotIO();
void LED2Tableii(led2);
void ReadLineSensor();
void bumperSwitchHandler();
void configPWMTimer(uint16_t, uint16_t, uint16_t, uint16_t);
void LEDControl();
void toggleAll();
void ProcessLineSensor();
void wheelsDirection(MotorState,MotorState);
void ControlRobot();
void StandBy();
void Robotlights();
int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    b0ButtonState = buttonOFF;

    config432IO();
    configRobotIO();

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); //ELB
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); //ERBR
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5); //DIRR
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
    toggleAll();

    while(1)
    {
        if(b0ButtonState == buttonON) //tracking
        {
            while(b0ButtonState == buttonON)
            {

                ReadLineSensor();
                ProcessLineSensor();
                LEDControl();
                ControlRobot();
                configPWMTimer(PERIOD, CLOCKDIVIDER, DUTYL, LEFTCHANNEL);
                configPWMTimer(PERIOD, CLOCKDIVIDER, DUTYR, RIGHTCHANNEL);
                Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
                Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
            }
        }

        else  //if bn is pressed
        {
            ReadLineSensor();
            ProcessLineSensor();
            LEDControl();
            wheelsDirection(off, off);

            __delay_cycles(750000);

            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

            __delay_cycles(750000);
        }
    }
}

// Function Name: configIO function
// Description: Configures to LEDs as outputs and sets them to low
// Input: None
// Return: None
// Author: Keila Souriac & Ruth Massock
void config432IO()
{
    /* Configuring P1.0 and P2.0 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); //Red Led 1
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2

    /*Set Red Led 1, Red led 2 Low, Green Led 2 Low, Blue Led 2 Low */
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red Led 1 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 Low
}

// Function Name: configRobotIO function
// Description: Configures to bumper switches as input with pull up resistors and interrupts
// Input: None
// Return: None
// Author: Keila Souriac & Ruth Massock
void configRobotIO()
{
    /* Configuring bumper switches as Input with pull up resistor */
    /* Configuring bumper switches as Input with pull up resistor */
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN0); //BMP0
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN2); //BMP1
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN3); //BMP2
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN5); //BMP3
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN6); //BMP4
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4,GPIO_PIN7); //BMP5

        GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN0); // clear interrupt flag
        GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN2); // clear interrupt flag
        GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN3); // clear interrupt flag
        GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN5); // clear interrupt flag
        GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN6); // clear interrupt flag
        GPIO_clearInterrupt(GPIO_PORT_P4,GPIO_PIN7); // clear interrupt flag

        MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN0);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN2);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN3);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN5);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN6);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P4,GPIO_PIN7);
    MAP_GPIO_registerInterrupt(GPIO_PORT_P4, bumperSwitchHandler);

    /* config RSLK LEDs as output & low */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN5); //Right Yellow LED front
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0); //Left Yellow LED
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN7); //Right Red LED back
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN6); //Left Red LED

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5); //Right yellow Led low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); //Left Yellow LED low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); //Right Red LED low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6); //Left Red LED low

    /* cntl even and odd */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3); //EVEN
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_PIN2); //ODD

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); //EVEN
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2); //ODD

    /* config motor connections */
    /* Configuring P1.0 and P2.0 as output */
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2); //ELB
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN4); //DIRL
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); //PWML
   // MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7); //PWML

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0); //ERB
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN5); //DIRR
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION); //PWMR
   // MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6); //PWMR

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); //ELBL
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); //DIRL
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //PWML

    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0); //ERBR
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); //DIRR
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //PWMR
}

// Function Name: LED2Tableii
// Description: This function controls the color display for LED2
// Input: none
// Return: None
// Author: Keila Souriac & Ruth Massock
void LED2Tableii(led2 LED2State)
{
    switch (LED2State)
    {

        case LED2OFF:
        // RED AND GREEN AND BLUE OFF
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 Low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 Low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 Low
            break;
        case RED:
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 High
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 Low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 Low
            break;

        case GREEN:
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 Low
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 High
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 Low
            break;

        case BLUE:
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 Low
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 Low
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 High
            break;

        case CYAN:
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 Low
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 High
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 High
            break;

        case YELLOW:
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 High
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 High
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 Low
            break;

        case WHITE:
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 High
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 High
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 High
          break;
    }
}

// Function Name: toggleAll
// Description: This function toggles all the MSP and RSLK LEDs once
// Input: none
// Return: None
// Author: Keila Souriac & Ruth Massock
void toggleAll()
{
    //RSLK LEDs High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5); //Right yellow Led High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0); //Left Yellow LED High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7); //Right Red LED High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6); //Left Red LED High

    //MSP LEDs High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red Led 1 High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 High
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 High

    __delay_cycles(6000000); //delay for 2 sec

    //RSLK LEDs Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5); //Right yellow Led low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); //Left Yellow LED low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); //Right Red LED low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6); //Left Red LED low

    //MSP LEDs Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red Led 1 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); //Red Led 2 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); //Green Led 2 Low
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); //Blue Led 2 Low
}

// Function Name: ReadLineSensor
// Description: reads the logic value of each sensor
// Input: none
// Return: None
// Author: Keila Souriac & Ruth Massock
void ReadLineSensor()
{
    uint8_t i;
    sensorNum = 0;

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); // Even IR LED
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P9, GPIO_PIN2); // Odd IR LED

    //set sensors as output and high
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN4 |GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7); //left sensors
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 |GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3); //right sensors
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4 |GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7); //left sensors
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0 |GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3); //right sensors

    __delay_cycles(30); // delay 30

    //config sensors as input
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4 |GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7); //left sensors
    MAP_GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN0 |GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3); //right sensors

    __delay_cycles(4000); // delay to read  4000

    // read each sensors value
    for (i = 0; i < 8; i++)
    {
        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P7, (1 << i)))
        {
        sensorNum |= (1 << i); // Set the corresponding bit in sensorNum
        }
    }
    // Turn off
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); // Turn off even
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_PIN2); // Turn off odd
}

// Function Name: ProcessLineSensor
// Description: This function counts how many left and right sensors are 1
// Input: none
// Return: None
// Author: Keila Souriac & Ruth Massock
void ProcessLineSensor()
{
    leftcount = 0;
    rightcount = 0;

    // Count how many sensors are logic 1
    if (sensorNum & BIT7) leftcount++;
    if (sensorNum & BIT6) leftcount++;
    if (sensorNum & BIT5) leftcount++;
    if (sensorNum & BIT4) leftcount++;

    if (sensorNum & BIT3) rightcount++;
    if (sensorNum & BIT2) rightcount++;
    if (sensorNum & BIT1) rightcount++;
    if (sensorNum & BIT0) rightcount++;

    // Check conditions for color classification
    if (leftcount == 4) // All sensors on the left are triggered
    {
        sensorCon = ALL_LEFT;  // RED
    }
    else if (rightcount == 4) // All sensors on the right are triggered
    {
        sensorCon = ONLYRIGHT;  // BLUE
    }
    else if (leftcount > rightcount) // More left sensors triggered (but not all left)
    {
        sensorCon = MORE_LEFT;  // YELLOW
    }
    else if (rightcount > leftcount) // More right sensors triggered (but not all right)
    {
        sensorCon = MORE_RIGHT;  // CYAN
    }
    else if (leftcount == rightcount && leftcount > 0) // Equal number of left and right sensors triggered
    {
        sensorCon = MIDDLE;  // GREEN
    }
    else if (leftcount == 0 && rightcount == 0) // No sensors are triggered
    {
        sensorCon = OFF;
    }
}


// Function Name: LEDControl
// Description: This function controls LED 1 and 2 depending on the sensors read.
// Input: none
// Return: None
// Author: Keila Souriac & Ruth Massock

void LEDControl()
{

    switch (sensorCon)
    {

        case ALL_LEFT:
            LED2Tableii(RED);
            break;

        case MORE_LEFT:
            LED2Tableii(YELLOW);
            break;

        case MIDDLE:
            LED2Tableii(GREEN);
            break;

        case MORE_RIGHT:
            LED2Tableii(CYAN);
            break;

        case ONLYRIGHT:
            LED2Tableii(BLUE);
            break;

        case OFF:
            LED2Tableii(WHITE);
            break;
    }

    if (sensorCon != OFF)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    }
}

// Function Name: bumperSwitchesHandler function
// Description: toggles the state of the bumper switches, form off to on
// Input: None
// Return: None
// Author: Keila Souriac & Ruth Massock
void bumperSwitchHandler()
{
    uint16_t status;
    _delay_cycles(30000); // Debounce delay
   status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P4);
    switch (status)
    {
        case GPIO_PIN3:
            if(b0ButtonState == buttonON)
            {
                b0ButtonState = buttonOFF;
                bnButtonState = buttonOFF;
            }
            else
            {
                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //Red Led 1 High
                b0ButtonState = buttonON;
                bnButtonState = buttonOFF;
            }
            break;
        default:
            if(bnButtonState == buttonON)
            {
                bnButtonState = buttonOFF;
                b0ButtonState = buttonOFF;
            }
            else
            {
                bnButtonState = buttonON;
                b0ButtonState = buttonOFF;

            }
            break;
    }
}

// Function Name: wheelsDircetion
// Description: This function controls the wheels direction (forwards, reverse, or off)
// Input: MotorState leftMotor, MotorState rightMotor
// Return: None
// Author: Keila Souriac & Ruth Massock

void wheelsDirection(MotorState leftMotor, MotorState rightMotor) {
    // Set left motor direction
    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); //ELBL

    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0); //ERBR

    if (leftMotor == Reverse)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN4); //DIRL
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL
    }
    else if (leftMotor == Forward)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN4); //DIRL
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL
    }
    else if (leftMotor == off)
    {
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); //SLPL
    }
    // Set right motor direction
    if (rightMotor == Reverse)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN5); //DIRR
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
    }
    else if (rightMotor == Forward)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN5); //DIRR
    }
    else if (rightMotor == off)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
    }
    else
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //SLPR
    }

    if(leftMotor == Reverse && rightMotor == Reverse)
    {
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN5); //Right yellow Led low
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); //Left Yellow LED low
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN7); //Right Red LED High
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN6); //Left Red LED High
    }
    else if(leftMotor == Forward && rightMotor == Forward)
    {
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN5); //Right yellow Led High
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0); //Left Yellow LED High
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN7); //Right Red LED low
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN6); //Left Red LED low
    }
}

// Function Name: ControlRobot
// Description: This function turns the RSLK robot
// Input: None
// Return: None
// Author: Keila Souriac & Ruth Massock
void ControlRobot()
{
//38,32(100)
    switch (sensorCon)
    {
        case ALL_LEFT: //red
            DUTYL = 35;
             DUTYR = 35;
           // DUTY = 35;  //32
            wheelsDirection(Reverse, Forward); //Hard Left Turn
            __delay_cycles(2500); //150000
            break;

        case MORE_LEFT: //yellow
            DUTYL = 15;
            DUTYR = 40;  //45
           wheelsDirection(Reverse, Forward); // slight left turn
           __delay_cycles(75);
           break;

        case MIDDLE: //green
            DUTYL = 42;
             DUTYR = 42;
          //  DUTY = 35; //32
            wheelsDirection(Forward, Forward);
            __delay_cycles(2500); //150000
            break;

      case MORE_RIGHT: //cyan
          DUTYR = 15;
          DUTYL = 40;
            //DUTY = 38;//45
            wheelsDirection(Forward, Reverse); // slight right turn
            __delay_cycles(75);
            break;

       case ONLYRIGHT: //blue
           DUTYL = 35;
            DUTYR = 35;
           // DUTY = 35; //32
           wheelsDirection(Forward, Reverse); //Hard Right Turn
            __delay_cycles(2500); //150000
            break;

        case OFF: //white
            DUTYL = 50;
             DUTYR = 50;
            //DUTY = 50;
            wheelsDirection(Reverse, Forward); //Hard Right Turn
            __delay_cycles(3000);
            break;
    }

}


// Function Name: configPWMTimer
// Description: This function uses the built in PWM mode
// Input: clockPeriod, clockDivider, duty, channel
// Return: None
// Author: This code was provided in the Lab 4 manual.
void configPWMTimer(uint16_t clockPeriod, uint16_t clockDivider, uint16_t duty, uint16_t channel)
{
    const uint32_t TIMER=TIMER_A0_BASE;
    uint16_t dutyCycle = duty*clockPeriod/100;
    timerPWMConfig.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    timerPWMConfig.clockSourceDivider = clockDivider;
    timerPWMConfig.timerPeriod = clockPeriod;
    timerPWMConfig.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_SET;
    timerPWMConfig.compareRegister = channel;
    timerPWMConfig.dutyCycle = dutyCycle;
    MAP_Timer_A_generatePWM(TIMER, &timerPWMConfig);
    MAP_Timer_A_stopTimer(TIMER);
}



