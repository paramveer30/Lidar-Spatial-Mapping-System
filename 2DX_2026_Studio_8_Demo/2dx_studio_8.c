//  Time of Flight Spatial Mapping  Project Deliverable 2
//  2DX3 McMaster University
//
//  Student Number: 400586406
//  Paramveer Multani
//
//  System Configuration:
//    Bus Speed       = 28 MHz  (PSYSDIV = 16, SysTick reload = 280000)
//    Measurement LED = PN0 (D2) — flashes on each distance reading
//    UART TX LED     = PF0 (D4) — flashes on serial transmission
//    Status LED      = PF4 (D3) — ON when idle/ready OFF during scan
//
//  Scan Parameters:
//    3 scans of 64 points each (5.625° angular resolution)
//    Alternating CW/CCW motor direction between scans
//    100 mm lateral offset between consecutive scans
//    Data transmitted over UART for python visualization

#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"


// I2C Control/Status Register Bit Masks

#define I2C_MCS_ACK             0x00000008  // Data acknowledge enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge address
#define I2C_MCS_STOP            0x00000004  // Generate STOP condition
#define I2C_MCS_START           0x00000002  // Generate START condition
#define I2C_MCS_ERROR           0x00000002  // Error flag
#define I2C_MCS_RUN             0x00000001  // I2C master enable
#define I2C_MCS_BUSY            0x00000001  // I2C busy flag
#define I2C_MCR_MFE             0x00000010  // I2C master function enable

#define MAXRETRIES              5           // Max I2C receive attempts before timeout


// Scan Configuration Constants
// 64 points per scan  360 / 64 = 5.625 degrees per measurement
// 32 motor steps per point  2048 total steps per revolution
// 100 mm offset between each scan plane along the x-axis

#define NUM_SCANS           3
#define POINTS_PER_SCAN     64
#define STEPS_PER_POINT     32
#define SCAN_OFFSET_MM      100

#define PJ0_MASK            0x01    // Bit mask for onboard switch SW1 (start)
#define PJ1_MASK            0x02    // Bit mask for onboard switch SW2 (pause)

// Global sensor address and status used across all functions
uint16_t    dev = 0x29;            // VL53L1X default I2C slave address
int status = 0;                    // Return status for API calls


// PortH_Init Stepper Motor Output (PH0–PH3)
// Configures Port H pins 0–3 as digital outputs for the four-phase stepper

void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                  // Enable Port H clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};       // Wait for clock ready
    GPIO_PORTH_DIR_R   |=  0x0F;    // PH0–PH3 as outputs
    GPIO_PORTH_AFSEL_R &= ~0x0F;    // Disable alternate functions
    GPIO_PORTH_DEN_R   |=  0x0F;    // Enable digital I/O
    GPIO_PORTH_AMSEL_R &= ~0x0F;    // Disable analog mode
    return;
}


// I2C_Init — I2C0 Master Configuration (PB2=SCL, PB3=SDA)

void I2C_Init(void){
    SYSCTL_RCGCI2C_R  |= SYSCTL_RCGCI2C_R0;                   // Enable I2C0 clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                   // Enable Port B clock
    while((SYSCTL_PRGPIO_R & 0x0002) == 0){};                   // Wait for Port B ready

    GPIO_PORTB_AFSEL_R |=  0x0C;    // Enable alternate function on PB2, PB3
    GPIO_PORTB_ODR_R   |=  0x08;    // Open-drain on PB3 (SDA)
    GPIO_PORTB_DEN_R   |=  0x0C;    // Digital enable PB2 PB3

    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFF00FF) + 0x00002200;  // Assign I2C0 to PB2/PB3
    I2C0_MCR_R  = I2C_MCR_MFE;                                  // Enable I2C0 master
    I2C0_MTPR_R = 0b0000000000000101000000000111011;             // Set SCL clock period
}


// PortG_Init VL53L1X XSHUT Control (PG0)
// Configures PG0 as a digital input by default
// Temporarily driven low by VL53L1X_XSHUT() to hardware-reset the ToF sensor

void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                  // Enable Port G clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R6) == 0){};       // Wait for clock ready
    GPIO_PORTG_DIR_R   &=  0x00;    // PG0 as input (default high-Z XSHUT pulled up)
    GPIO_PORTG_AFSEL_R &= ~0x01;    // Disable alternate function
    GPIO_PORTG_DEN_R   |=  0x01;    // Digital enable
    GPIO_PORTG_AMSEL_R &= ~0x01;    // Disable analog mode
    return;
}


// VL53L1X_XSHUT Hardware Reset of ToF Sensor
// Pulls XSHUT (PG0) low to power-down the sensor waits 100 ms
// then releases the pin to allow the sensor to reboot

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |=  0x01;       // Set PG0 as output
    GPIO_PORTG_DATA_R &= 0b11111110; // Drive XSHUT low (sensor off)
    FlashAllLEDs();                  // Visual indicator during reset
    SysTick_Wait10ms(10);            // Hold low for 100 ms
    GPIO_PORTG_DIR_R &= ~0x01;       // Release PG0 back to input (sensor reboots)
}


// spinCW Rotate Stepper Motor Clockwise
// Coil pattern: AB  BC  CD  DA (full-step drive)

void spinCW(uint32_t steps, uint32_t delay){
    for(int i = 0; i < steps; i++){
        GPIO_PORTH_DATA_R = 0b00000011;     // Phase 1: coils A + B
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00000110;     // Phase 2: coils B + C
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00001100;     // Phase 3: coils C + D
        SysTick_Wait10ms(delay);
        GPIO_PORTH_DATA_R = 0b00001001;     // Phase 4: coils D + A
        SysTick_Wait10ms(delay);
    }
}


// Volatile Flags Shared Between ISR and Main Context
// startRequest  : set by PJ0 ISR, consumed by main loop to trigger a scan
// pauseRequest  : set by PJ1 ISR, consumed by spinOnePoint to toggle pause
// motorPaused   : tracks whether motor is currently paused
// scanRunning   : prevents re-entry while a scan is in progress

volatile uint8_t startRequest = 0;
volatile uint8_t pauseRequest = 0;
volatile uint8_t motorPaused  = 0;
volatile uint8_t scanRunning  = 0;

uint8_t  currentScan       = 0;     // Index of next scan to run (0, 1, 2)
uint8_t  currentDirection  = 1;     // 1 = clockwise 0 = counter-clockwise
uint16_t distances[NUM_SCANS][POINTS_PER_SCAN];  // Stored distance readings (mm)


// PortJ_Init Onboard Button Inputs (PJ0=SW1, PJ1=SW2)
// Configures both buttons as digital inputs with internal pull-up resistors
// Buttons are active-low (pressed = logic 0)

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                  // Enable Port J clock
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};       // Wait for clock ready
    GPIO_PORTJ_DIR_R   &= ~(PJ0_MASK | PJ1_MASK);  // PJ0, PJ1 as inputs
    GPIO_PORTJ_DEN_R   |=  (PJ0_MASK | PJ1_MASK);  // Digital enable
    GPIO_PORTJ_PUR_R   |=  (PJ0_MASK | PJ1_MASK);  // Internal pull-ups (active-low buttons)
    GPIO_PORTJ_AFSEL_R &= ~(PJ0_MASK | PJ1_MASK);  // Disable alternate functions
    GPIO_PORTJ_AMSEL_R &= ~(PJ0_MASK | PJ1_MASK);  // Disable analog mode
}


// PortJ_Interrupt_Init Falling-Edge GPIO Interrupt for PJ0 and PJ1
// Detects button presses on both switches
// Port J maps to IRQ 51 (bit 19 of NVIC_EN1_R) priority set to 5

void PortJ_Interrupt_Init(void){
    GPIO_PORTJ_IS_R  &= ~(PJ0_MASK | PJ1_MASK);   // Edge-sensitive prevents spamming when held
    GPIO_PORTJ_IBE_R &= ~(PJ0_MASK | PJ1_MASK);   // Single-edge triggered
    GPIO_PORTJ_IEV_R &= ~(PJ0_MASK | PJ1_MASK);   // Falling edge (button press)
    GPIO_PORTJ_ICR_R  =  (PJ0_MASK | PJ1_MASK);   // Clear any pending interrupts
    GPIO_PORTJ_IM_R  |=  (PJ0_MASK | PJ1_MASK);   // Unmask interrupts on PJ0 PJ1
    NVIC_EN1_R  |= 0x00080000;                      // Enable IRQ 51 in NVIC
    NVIC_PRI12_R = (NVIC_PRI12_R & 0x00FFFFFF) | 0xA0000000;  // Set priority 5
    __asm(" cpsie i");                               // Global interrupt enable
}


// GPIOJ_IRQHandler Port J Interrupt Service Routine
// Fires on falling edge on PJ0 or PJ1
// Sets request flag and clears interrupt, processing deferred to main loop

void GPIOJ_IRQHandler(void){
    if(GPIO_PORTJ_MIS_R & PJ0_MASK){       // SW1 pressed request scan start
        startRequest = 1;
        GPIO_PORTJ_ICR_R = PJ0_MASK;       // Clear PJ0 interrupt flag
    }
    if(GPIO_PORTJ_MIS_R & PJ1_MASK){       // SW2 pressed request pause toggle
        pauseRequest = 1;
        GPIO_PORTJ_ICR_R = PJ1_MASK;       // Clear PJ1 interrupt flag
    }
}


// motorOff  De-energize All Stepper Coils
// Writes 0x00 to Port H turning off all four coil outputs
// Called between scans to prevent motor heating while idle

void motorOff(void){
    GPIO_PORTH_DATA_R = 0x00;
}


// setReadyLED — Control Status LED on PF4 (D3)
// on = 1: LED ON  (system idle waiting for button press)
// on = 0: LED OFF (scan in progress)

void setReadyLED(int on){
    if(on){
        GPIO_PORTF_DATA_R |=  0x10;        // Set PF4 high
    } else {
        GPIO_PORTF_DATA_R &= ~0x10;        // Clear PF4
    }
}


// spinOnePoint Advance Motor by One Measurement Increment (5.625°)
// Steps motor STEPS_PER_POINT (32) times using 4-phase full-step sequence
// Checks pause flag between each step allowing freeze/resume through SW2

void spinOnePoint(int isCW){
    static uint8_t seqIdx = 0;             // Remember position in coil sequence
    const uint8_t seq[4] = {0x0C, 0x06, 0x03, 0x09};
    int s;

    for(s = 0; s < STEPS_PER_POINT; s++){

        // Handle pause/resume toggle from SW2 interrupt
        if(pauseRequest){
            pauseRequest = 0;
            motorPaused = !motorPaused;
            UART_printf(motorPaused ? "Paused\r\n" : "Resumed\r\n");
        }
        // Spin-wait while paused, resume on next SW2 press
        while(motorPaused){
            if(pauseRequest){
                pauseRequest = 0;
                motorPaused = 0;
                UART_printf("Resumed\r\n");
            }
        }

        // Advance sequence index based on direction
        if(isCW){
            seqIdx = (seqIdx + 1) % 4;
        } else {
            seqIdx = (seqIdx - 1) & 3;
        }
        GPIO_PORTH_DATA_R = seq[seqIdx];   // Energize next coil pair
        SysTick_Wait10ms(2);               // 20 ms settling delay per step
    }
}


// runOneScan Execute a Full 360° Scan (64 Distance Measurements)

void runOneScan(uint8_t scanNum){
    uint8_t  dataReady = 0;
    uint16_t Distance;
    uint8_t  RangeStatus;
    uint32_t angle_mdeg;

    scanRunning = 1;
    setReadyLED(0);                 // Status LED off while scanning

    // Transmit scan header data for the parser
    sprintf(printf_buffer, "BEGIN_SCAN_%u\r\n", scanNum + 1);
    UART_printf(printf_buffer);

    sprintf(printf_buffer, "SCAN_INDEX,%u\r\n", scanNum + 1);
    UART_printf(printf_buffer);

    sprintf(printf_buffer, "X_MM,%u\r\n", (unsigned)(scanNum * SCAN_OFFSET_MM));
    UART_printf(printf_buffer);

    sprintf(printf_buffer, "Direction: %s\r\n", currentDirection ? "CW" : "CCW");
    UART_printf(printf_buffer);

    for(int i = 0; i < POINTS_PER_SCAN; i++){

        // Step 1: Rotate motor to the next angular position
        spinOnePoint(currentDirection);

        // Step 2: Poll sensor until a new measurement is available
        dataReady = 0;
        while(dataReady == 0){
            status = VL53L1X_CheckForDataReady(dev, &dataReady);
            FlashLED3(1);
            VL53L1_WaitMs(dev, 5);
        }

        // Step 3: Read range status and distance value
        status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
        status = VL53L1X_GetDistance(dev, &Distance);

        FlashLED4(1);               // Flash UART TX LED on data read

        // Step 4: Clear sensor interrupt to allow next measurement
        status = VL53L1X_ClearInterrupt(dev);

        // Step 5: Store reading and transmit over UART
        distances[scanNum][i] = Distance;

        FlashLED2(1);               // Flash measurement LED (PN0)

        // Calculate angle: 64 points across 360 degrees = 5625 milli-degrees per step
        angle_mdeg = i * 5625;

        sprintf(printf_buffer, "Point %d, Offset: %u mm, Angle: %lu mdeg, Distance: %u mm\r\n",
            i + 1,
            (unsigned)(scanNum * SCAN_OFFSET_MM),
            (unsigned long)angle_mdeg,
            Distance
        );
        UART_printf(printf_buffer);
    }

    motorOff();                      // De-energize coils after scan

    currentDirection = !currentDirection;  // Alternate direction for next scan

    scanRunning = 0;
    setReadyLED(1);                  // Status LED on scan complete

    sprintf(printf_buffer, "END_SCAN_%u\r\n", scanNum + 1);
    UART_printf(printf_buffer);

    sprintf(printf_buffer, "Scan %u complete\r\n", scanNum + 1);
    UART_printf(printf_buffer);
}


// sendAllData Retransmit All Stored Measurements Over UART

void sendAllData(void){
    int scan, i;
    uint32_t angle_mdeg;

    FlashLED4(1);                    // Flash UART TX LED at start of bulk send

    UART_printf("START\r\n");

    for(scan = 0; scan < NUM_SCANS; scan++){

        sprintf(printf_buffer, "SCAN_INDEX,%d\r\n", scan + 1);
        UART_printf(printf_buffer);

        sprintf(printf_buffer, "X_MM,%d\r\n", scan * SCAN_OFFSET_MM);
        UART_printf(printf_buffer);

        for(i = 0; i < POINTS_PER_SCAN; i++){

            angle_mdeg = i * 5625;

            sprintf(printf_buffer, "Point %d, Offset: %d mm, Angle: %lu mdeg, Distance: %u mm\r\n",
                i + 1,
                scan * SCAN_OFFSET_MM,
                (unsigned long)angle_mdeg,
                distances[scan][i]
            );
            UART_printf(printf_buffer);
        }
    }

    UART_printf("END\r\n");
    UART_printf("All data sent\r\n");
}


// MAIN FUNCTION

int main(void) {

    // Sensor communication variables
    uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, i=0;
    uint16_t wordData;
    uint16_t Distance;
    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum;
    uint8_t RangeStatus;
    uint8_t dataReady;

    // Peripheral Initialization
    PLL_Init();                      // Configure system clock to 28 MHz
    SysTick_Init();                  // Initialize SysTick timer for delays
    onboardLEDs_Init();              // Enable onboard LED GPIO pins
    I2C_Init();                      // Configure I2C0 master for ToF sensor
    UART_Init();                     // Configure UART for serial output
    PortG_Init();                    // Configure PG0 for XSHUT control
    PortH_Init();                    // Configure PH0–PH3 for stepper motor
    PortJ_Init();                    // Configure PJ0/PJ1 as button inputs
    PortJ_Interrupt_Init();          // Enable falling-edge interrupts on PJ0/PJ1

    // Startup Messages
    UART_printf("Program Begins\r\n");
    int mynumber = 1;
    sprintf(printf_buffer, "2DX ToF Program Studio Code %d\r\n", mynumber);
    UART_printf(printf_buffer);
    sprintf(printf_buffer, "Student 400586406 | 28MHz | Meas:PN0 | UART:PF0 | Ready:PF4\r\n");
    UART_printf(printf_buffer);

    // Read and display sensor model ID to verify I2C communication
    status = VL53L1X_GetSensorId(dev, &wordData);
    sprintf(printf_buffer, "(Model_ID, Module_Type)=0x%x\r\n", wordData);
    UART_printf(printf_buffer);

    // ToF Sensor Boot Sequence

    // Wait for the VL53L1X to complete its internal boot
    while(sensorState == 0){
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    FlashAllLEDs();
    UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");

    // Clear any residual interrupt from a previous session
    status = VL53L1X_ClearInterrupt(dev);

    // Load default calibration and configuration into the sensor
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);

    // Configure ranging parameters
    status = VL53L1X_SetDistanceMode(dev, 2);           // Long-range mode (up to 4 m)
    status = VL53L1X_SetTimingBudgetInMs(dev, 50);      // 50 ms measurement window
    status = VL53L1X_SetInterMeasurementInMs(dev, 60);  // 60 ms between measurements

    // Begin continuous ranging
    status = VL53L1X_StartRanging(dev);
    Status_Check("StartRanging", status);

    setReadyLED(1);                  // Status LED on system ready
    UART_printf("System ready. Press PJ0 (SW1) to start scan 1.\r\n");

    // Main Loop Event-Driven Scan Controller

    while(1){
        if(!scanRunning && startRequest){
            startRequest = 0;

            if(currentScan < NUM_SCANS){
                runOneScan(currentScan);
                currentScan++;

                if(currentScan < NUM_SCANS){
                    // Prompt for next scan
                    sprintf(printf_buffer, "Press PJ0 for scan %u\r\n", currentScan + 1);
                    UART_printf(printf_buffer);
                } else {
                    // All scans complete transmit stored data and reset
                    UART_printf("All scans complete\r\n");
                    sendAllData();
                    VL53L1X_StopRanging(dev);        // Stop continuous ranging
                    currentScan = 0;                 // Reset scan counter for next run
                    setReadyLED(1);
                    UART_printf("Done. Press PJ0 to scan again.\r\n");
                    status = VL53L1X_StartRanging(dev);  // Re-arm ranging for next session
                }
            }
        }
    }
}