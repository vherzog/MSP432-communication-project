/*********************************************************************************
* ENG EC 450 -- FINAL PROJECT -- DOODLE TALKIES
* Team members: Veronica Herzog, Blake Hina
*********************************************************************************/

// Includes:
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

// Global Variables:
Graphics_Context g_sContext;        // LCD display
static uint16_t resultsBuffer[2];   // ADC results buffer

// DISPLAY:
volatile int Position_A_X;          // Position of cursor x-coordinate
volatile int Position_A_Y;          // Position of cursor y-coordinate
volatile char Position_A_X_Out;     // Position direction x-coordinate
volatile char Position_A_Y_Out;     // Position direction y-coordinate

// DEBOUNCING:
volatile int delay_max=1;
volatile int delay_x;
volatile int delay_y;
volatile int delay_x_In;
volatile int delay_y_In;

// COMMUNICATION:
#define MAX 1000                    // Maximum length of send/receive array
// SENDING DATA:
volatile int noMessageSend;         // message ready to send state
volatile int sendState;             // 0: 'M', 1: x,y sending
volatile int coordState;            // toggle through sending x,y pairs
volatile int sendArray[2][MAX];     // saved coordinates of your message
volatile int sendArrayCount;        // length of your message
volatile int sendIndex;             // length of your message (used for iterating)
// RECEIVING DATA:
volatile int noMessageRec;          // message received state
volatile char recByte;              // received data byte
volatile char recByte_last;         // last received data byte
volatile int recArray[2][MAX];      // saved coordinates of received message
volatile int recArrayCount;         // length of received message
volatile int recArrayIndex;         // length of received message (used for iterating)

// SYSTEM STATES:
volatile int drawMode;              // draw mode used to pick up cursor
volatile int sendMessageMax;        // Reached the maximum send array size



//****************************************************************************


/* UART CONFIGURATION */
const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
    78,                                     // BRDIV = 78
    2,                                       // UCxBRF = 2
    0,                                       // UCxBRS = 0
    EUSCI_A_UART_NO_PARITY,                  // No Parity
    EUSCI_A_UART_LSB_FIRST,                  // MSB First
    EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
    EUSCI_A_UART_MODE,                       // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

/*************************************************
 * ADC14 Subsystem
 *************************************************/
/*
 * ADC Interrupt handler
 * This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
 */
void ADC14_IRQHandler(void){
    //****************************************************************************
    //  Interrupt called when every Analog to Digital Conversion is available
        uint64_t status;
        status = MAP_ADC14_getEnabledInterruptStatus();
        MAP_ADC14_clearInterruptFlag(status);
            //  Enable interrupt and clear flag

        /* ADC_MEM1 conversion completed */
        if(status & ADC_INT1)
        {
            // x and y coordinates of Joystick
            resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
            resultsBuffer[1] = ADC14_getResult(ADC_MEM1);

            // If the maximum message length has been exceeded, no longer allow drawing
            if (sendArrayCount > (MAX-1)) {
                drawMode = 0;
                sendMessageMax = 1;
            }

            // Print your score on to display using char array & sprintf
            char string_print[15];
            if (drawMode) {
                sprintf(string_print, "Draw Mode: ON \n");
            }
            else if(sendMessageMax) {
                sprintf(string_print, "Max Message Reached\n");
            }
            else {
                sprintf(string_print, "Draw Mode: OFF\n");
            }
            Graphics_drawStringCentered(&g_sContext,(int8_t *)string_print,AUTO_STRING_LENGTH,64,10,OPAQUE_TEXT);

            // Bounds of actual dots on screen
            int X_Max=120;
            int X_Min=8;
            int Y_Max=120;
            int Y_Min=20;

            // Bounds of dot on screen
            // A:
            if(Position_A_X>X_Max){
                Position_A_X=X_Max;
            }
            if(Position_A_X<X_Min){
                Position_A_X=X_Min;
            }
            if(Position_A_Y>Y_Max){
                Position_A_Y=Y_Max;
            }
            if(Position_A_Y<Y_Min){
                Position_A_Y=Y_Min;
            }
            // B:
            if(Position_B_X>X_Max){
                Position_B_X=X_Max;
            }
            if(Position_B_X<X_Min){
                Position_B_X=X_Min;
            }
            if(Position_B_Y>Y_Max){
                Position_B_Y=Y_Max;
            }
            if(Position_B_Y<Y_Min){
                Position_B_Y=Y_Min;
            }

            // Approx. default position of Joystick
            int X_zero=8200;
            int Y_zero=8200;
            int threshold_x=1000;
            int threshold_y=1000;

            // FOR X:
            Position_A_X_last = Position_A_X; // save last position to possibly clear
            //  Reset delay counter when joystick is in stationary range
            if(X_zero-threshold_x<resultsBuffer[0] && resultsBuffer[0]<X_zero+threshold_x){
                delay_x=delay_max;
                Position_A_X_Out='S';
            }
            // Decrements delay counter when joystick is in forward stage
            if(resultsBuffer[0]>X_zero+threshold_x){
                delay_x=delay_x-1;
                if(delay_x==0){
                    Position_A_X=Position_A_X+1;
                    Position_A_X_Out='F';
                    delay_x=delay_max;
                }
            }
            // Decrements delay counter when joystick is in backward stage
            if(resultsBuffer[0]<X_zero-threshold_x){
                delay_x=delay_x-1;
                if(delay_x==0){
                    Position_A_X=Position_A_X-1;
                    Position_A_X_Out='B';
                    delay_x=delay_max;
                }
            }

            // FOR Y (same as above):
            Position_A_Y_last = Position_A_Y; // save last position to possibly clear
            if(Y_zero-threshold_y<resultsBuffer[1] && resultsBuffer[1]<Y_zero+threshold_y){
                delay_y=delay_max;
                Position_A_Y_Out='S';
            }
            if(resultsBuffer[1]>Y_zero+threshold_y){
                delay_y=delay_y-1;
                if(delay_y==0){
                    Position_A_Y=Position_A_Y-1;
                    Position_A_Y_Out='F';
                    delay_y=delay_max;
                }
            }
            if(resultsBuffer[1]<Y_zero-threshold_y){
                delay_y=delay_y-1;
                if(delay_y==0){
                    Position_A_Y=Position_A_Y+1;
                    Position_A_Y_Out='B';
                    delay_y=delay_max;
                }
            }

            // DISPLAY:
            //  A dot
            if (!drawMode) {
                Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
                Graphics_fillCircle(&g_sContext,Position_A_X_last,Position_A_Y_last,2);
            }

            if(noMessageSend && drawMode) {
                // SAVE THE X,Y COORDINATES TO VECTOR
                if((Position_A_X != 67) && (Position_A_X != 77) && (Position_A_X != 78) && (Position_A_Y != 67) && (Position_A_Y != 77) && (Position_A_Y != 78) && (Position_A_X_Out != 'S' || Position_A_Y_Out != 'S')){
                    sendArray[0][sendArrayCount] = Position_A_X;
                    sendArray[1][sendArrayCount] = Position_A_Y;
                    sendArrayCount++;
                }
            }
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
            Graphics_fillCircle(&g_sContext,Position_A_X,Position_A_Y,2);

    }
}

//// BUTTONS -- using WDT:
void change_drawState() {
    drawMode = drawMode ? 0 : 1;
}

void clear_recArray() {
    volatile int i;
    for(i = 0; i < recArrayIndex; i++) {
        Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
        Graphics_fillCircle(&g_sContext,recArray[0][i],recArray[1][i],2);
        recArray[0][i] = -1;
        recArray[1][i] = -1;
    }
    recArrayIndex = 0;
}

void clear_sendArray() {
    volatile int i;
    for(i = 0; i < sendArrayCount; i++){

        sendArray[0][i] = -1;
        sendArray[1][i] = -1;
    }
    sendArrayCount = 0;
}

void clear_screen() {
    clear_sendArray();
    sendMessageMax = 0;

    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
}


/*
 * Watchdog Timer interrupt service routine
 * (Note: the function name for this handler is declared as an external routine in one of the
 * automatically loaded startup c files from the Code Composer system).
 */
volatile unsigned char last_button35;
volatile unsigned char last_button51;
volatile unsigned char last_button55;
volatile unsigned char last_button54;

void WDT_A_IRQHandler(void) {
    signed int button35;
    signed int button51;
    signed int button55;
    signed int button54;
    /* Check if button was pressed since last interrupt */
    button35 = P3->IN & BIT5; // toggle drawMode
    button51 = P5->IN & BIT1; // clear screen
    button55 = P5->IN & BIT5; // send message
    button54 = P5->IN & BIT4; // clear send message

    // turns off Draw Mode
    if((button35 == 0) && last_button35) {
        // If pressed to unpressed
        change_drawState();
    }

    // Clears the screen
    if((button51 == 0) && last_button51) {
        clear_screen();
    }

    // Clears received drawing
    if((button54 == 0) && last_button54) {
        clear_recArray();
    }

    // Sends your message
    if((button55 == 0) && last_button55) {
        noMessageSend = 0;
        drawMode = 0;
    }
    last_button35 = button35;
    last_button51 = button51;
    last_button55 = button55;
    last_button54 = button54;
}



/*************************************************
 * UART PROTOCOL HANDLER
 *************************************************/
volatile int x_pos;
void EUSCIA2_IRQHandler(void){
    // Current received byte
    recByte = UCA2RXBUF;

    /* TRANSLATE RECIEVED DATA: */
    // There is no message being sent from partner
    if(recByte == 'N') {
        noMessageRec = 1;
    }
    // There is a message being sent
    else if(recByte_last == 'M') {
        noMessageRec = 0;
        recArrayCount = 0;
    }

    // Translate and plot x & y coordinates of received message
    if (!noMessageRec) {
        if(recByte=='X'){
            // ADD x-coordinate to RECEIVED DRAWING
            x_pos = recByte_last;
        }
        else if(recByte=='Y'){
            // ADD y-coordinate to RECEIVED DRAWING and send the coordinate to display!
            Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLUE);
            Graphics_fillCircle(&g_sContext,x_pos,recByte_last,2);
            recArray[0][recArrayCount] = x_pos;
            recArray[1][recArrayCount] = recByte_last;
            recArrayCount++;
        }
        else if(recByte == 'C'){
            noMessageRec = 1;
            recArrayIndex = recArrayCount;
        }

    }

    // Save last byte received
    recByte_last = recByte;



    /* TRANSLATE SENDING DATA */
    // If there is no message to send yet
    if(noMessageSend) {
        UART_transmitData(EUSCI_A2_BASE, 'N');
    }
    // If there is a message to send
    else {
        if(sendState == 0) {
            // Indicate a message is ready to be sent
            UART_transmitData(EUSCI_A2_BASE, 'M');
            // Sending x,y pairs, initialize variables
            sendState = 1;
        }
        else if(sendState == 1) {
            // Indicate length of Array sending:
            UART_transmitData(EUSCI_A2_BASE, sendArrayCount);
            sendIndex = 0;
            sendState = 2;
        }
        else if(sendState == 2 ){
            // Iterate through (x,y) pairs of send message array until complete
            if (sendIndex < sendArrayCount){
                if(coordState == 0) {
                    // send the current x-coordinate
                    UART_transmitData(EUSCI_A2_BASE, sendArray[0][sendIndex]);
                    coordState=coordState+1;
                }
                else if(coordState == 1) {
                    UART_transmitData(EUSCI_A2_BASE, 'X');
                    coordState=coordState+1;
                }
                else if(coordState==2) {
                    // send the current y-coordinate
                    UART_transmitData(EUSCI_A2_BASE, sendArray[1][sendIndex]);
                    coordState=coordState+1;
                }
                else {
                    UART_transmitData(EUSCI_A2_BASE, 'Y');
                    coordState=0;
                    sendIndex++;
                }
            }
            // When done iterating, indicate message complete
            else {
                // Indicate a message is completed
                UART_transmitData(EUSCI_A2_BASE, 'C');
                // Message complete, set states back to default
                noMessageSend = 1;
                sendState = 0;
                clear_sendArray();
            }
        }
    }
}


void main(void){

    // SET UP GPIO PINS for UART and DISPLAY:
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
    GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
        GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    // Using DCO clock 12MHz
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

    //UART
    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfig);
    MAP_UART_enableModule(EUSCI_A2_BASE);

    // Interrupt enables:
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_enableMaster();

    // set delays to max
    delay_x=delay_max;
    delay_y=delay_max;
    delay_x_In=delay_max;
    delay_y_In=delay_max;

    // Initialize location of your cursor
    Position_A_X=70;
    Position_A_Y=10;

    // Initialize cursor to stationary
    Position_A_X_Out='S';
    Position_A_Y_Out='S';

    // Initialize transmitted bytes
    recByte='N';
    recByte_last='N';

    // Initialize system states
    drawMode = 1;
    // Sending Data
    noMessageSend = 1;
    sendState = 0;
    coordState = 0;
    sendArrayCount = 0;
    sendIndex = 0;
    sendMessageMax = 0;
    // Receiving Data
    noMessageRec = 1;
    x_pos = 0;

    // Initialize two message arrays to -1
    volatile int j;
    for(j = 0; j < MAX; j++) {
        sendArray[0][j] = -1;
        sendArray[1][j] = -1;
        recArray[0][j] = -1;
        recArray[1][j] = -1;
    }


    /* Timers */
    WDT_A->CTL = WDT_A_CTL_PW |             // 'password' to enable access
                 WDT_A_CTL_SSEL__SMCLK |         // clock source = SMCLK
                 WDT_A_CTL_TMSEL |               // this bit turns on interval mode
                 WDT_A_CTL_CNTCL |               // clear the internal WDT counter
                 WDT_A_CTL_IS_5;

    
    // Initialize 4 Buttons
    P3->DIR &=~BIT5;   // clear the direction
    last_button35 = P3->IN & BIT5; // initialize the previous state of the button (needed by handler)

    P5->DIR &=~BIT1;
    last_button51 = P5->IN & BIT1;

    P5->DIR &=~BIT5;
    last_button55 = P5->IN & BIT5;

    P5->DIR &=~BIT4;
    last_button54 = P5->IN & BIT4;


    // setup so that the system sleeps until interrupted
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Specify that after an interrupt, the CPU wakes up

    __enable_interrupt();                   // allow the CPU to respond to interrupt signals.
    NVIC->ISER[0] = 1 << ((WDT_A_IRQn) & 31); // enable WDT to send interrupt signals

    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);


    // Initializes Clock System
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);


    // Initializes display
    Crystalfontz128x128_Init();

    // Set default screen orientation
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    // Initializes graphics context
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,&g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);


    // Configures Pin 6.0 and 4.4 as ADC input 
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    // Initializing ADC (ADCOSC/64/8) 
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    // Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
    // with internal 2.5v reference 
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);
    MAP_ADC14_configureConversionMemory(ADC_MEM1,ADC_VREFPOS_AVCC_VREFNEG_VSS,ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    // Enabling the interrupt when a conversion on channel 1 (end of sequence)
    //  is complete and enabling conversions
    MAP_ADC14_enableInterrupt(ADC_INT1);

    // Enabling Interrupts 
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    // Setting up the sample timer to automatically step through the sequence convert.
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    // Triggering the start of the sample
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    UART_transmitData(EUSCI_A2_BASE, 'N');

    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}
