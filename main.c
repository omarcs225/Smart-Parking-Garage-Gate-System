#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "tm4c123gh6pm.h"
#include <stdint.h>
#include <stdbool.h>
#include "basic_io.h"

#define MAX_PARKING_SPOTS       5
#define GATE_OPEN_TIME_MS       2000

#define GREEN_LED   (1U << 3)
#define RED_LED     (1U << 1)
#define BLUE_LED    (1U << 2)
#define SW1         (1U << 4)
#define SW2         (1U << 0)

typedef enum {
    GATE_IDLE_CLOSED,
    GATE_IDLE_OPEN,
    GATE_OPENING,
    GATE_CLOSING,
    GATE_STOPPED_OBSTACLE
} GateState_t;

SemaphoreHandle_t xObstacleSemaphore;				//binary
SemaphoreHandle_t xEntrySemaphore;					//binary
SemaphoreHandle_t xExitSemaphore;					//binary - signals exit task when car enters
SemaphoreHandle_t xParkingSpotsSemaphore;		//counting
SemaphoreHandle_t xGateStateMutex;					//mutex

volatile GateState_t gateState = GATE_IDLE_CLOSED;

// Helper functions for clean mutex usage
static void GateState_Set(GateState_t newState) {
    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
    gateState = newState;
    xSemaphoreGive(xGateStateMutex);
}

static GateState_t GateState_Get(void) {
    GateState_t state;
    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
    state = gateState;
    xSemaphoreGive(xGateStateMutex);
    return state;
}

static bool GateState_CompareAndSet(GateState_t expected, GateState_t newState) {
    bool success = false;
    xSemaphoreTake(xGateStateMutex, portMAX_DELAY);
    if (gateState == expected) {
        gateState = newState;
        success = true;
    }
    xSemaphoreGive(xGateStateMutex);
    return success;
}

void Hardware_Init(void) {
    SYSCTL_RCGCGPIO_R |= 0x20;
    while ((SYSCTL_PRGPIO_R & 0x20) == 0) {}
    
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= 0x01;
    
    GPIO_PORTF_DIR_R |= (GREEN_LED | RED_LED | BLUE_LED);
    GPIO_PORTF_DEN_R |= (GREEN_LED | RED_LED | BLUE_LED | SW1 | SW2);
    GPIO_PORTF_DIR_R &= ~(SW1 | SW2);
    GPIO_PORTF_PUR_R |= (SW1 | SW2);
    
    // Configure SW1 and SW2 for edge-triggered interrupts (falling edge)
    GPIO_PORTF_IS_R &= ~(SW1 | SW2);   // Edge-sensitive
    GPIO_PORTF_IBE_R &= ~(SW1 | SW2);  // Not both edges
    GPIO_PORTF_IEV_R &= ~(SW1 | SW2);  // Falling edge (button press)
    GPIO_PORTF_ICR_R = (SW1 | SW2);    // Clear any prior interrupts
    GPIO_PORTF_IM_R |= (SW1 | SW2);    // Enable interrupts for both switches
    
    NVIC_EN0_R |= (1 << 30);
    NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | (5 << 21);
}

void LED_Set(uint8_t led, bool on) {
    if (on) GPIO_PORTF_DATA_R |= led;
    else GPIO_PORTF_DATA_R &= ~led;
}

void LED_AllOff(void) {
    GPIO_PORTF_DATA_R &= ~(GREEN_LED | RED_LED | BLUE_LED);
}

void GPIOF_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (GPIO_PORTF_RIS_R & SW1) {
        GPIO_PORTF_ICR_R = SW1;
        xSemaphoreGiveFromISR(xObstacleSemaphore, &xHigherPriorityTaskWoken);
    }
    
    if (GPIO_PORTF_RIS_R & SW2) {
        GPIO_PORTF_ICR_R = SW2;
        xSemaphoreGiveFromISR(xEntrySemaphore, &xHigherPriorityTaskWoken);
    }
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void vSafetyTask(void *pvParameters) {
    (void)pvParameters;
    vPrintString("Safety Task Started\n");
    
    for (;;) {
        if (xSemaphoreTake(xObstacleSemaphore, portMAX_DELAY) == pdTRUE) {
            if (GateState_CompareAndSet(GATE_CLOSING, GATE_STOPPED_OBSTACLE)) {
                vPrintString("OBSTACLE DETECTED! Gate stopped.\n");
                LED_AllOff();
                LED_Set(GREEN_LED, true);
                vTaskDelay(pdMS_TO_TICKS(500));
                LED_AllOff();
            }
        }
    }
}

void vEntryTask(void *pvParameters) {
    (void)pvParameters;
    vPrintString("Entry Task Started\n");
    
    for (;;) {
        if (xSemaphoreTake(xEntrySemaphore, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(xParkingSpotsSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                GateState_Set(GATE_OPENING);
                vPrintStringAndNumber("Car entering. Spots left:", uxSemaphoreGetCount(xParkingSpotsSemaphore));
                
                LED_AllOff();
                LED_Set(GREEN_LED, true);
                // Signal exit task that a car has entered
                xSemaphoreGive(xExitSemaphore);
                vTaskDelay(pdMS_TO_TICKS(GATE_OPEN_TIME_MS));
                
                GateState_Set(GATE_CLOSING);
                vPrintString("Gate closing...\n");
                
                LED_AllOff();
                LED_Set(RED_LED, true);
                vTaskDelay(pdMS_TO_TICKS(GATE_OPEN_TIME_MS));
                LED_AllOff();
                
                GateState_Set(GATE_IDLE_CLOSED);
            } else {
                vPrintString("Parking FULL! Entry denied.\n");
                LED_Set(RED_LED, true);
                vTaskDelay(pdMS_TO_TICKS(100));
                LED_Set(RED_LED, false);
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            
            // Debounce: wait for button release
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void vExitTask(void *pvParameters) {
    (void)pvParameters;
    vPrintString("Exit Task Started\n");
    
    for (;;) {
        // Wait for entry task to signal that a car has entered
        if (xSemaphoreTake(xExitSemaphore, portMAX_DELAY) == pdTRUE) {
            // Simulate car staying in parking for 10 seconds
            vTaskDelay(pdMS_TO_TICKS(10000));
            
            xSemaphoreGive(xParkingSpotsSemaphore);
            vPrintStringAndNumber("Car exited. Spots available:", uxSemaphoreGetCount(xParkingSpotsSemaphore));
            LED_Set(GREEN_LED, true);
            vTaskDelay(pdMS_TO_TICKS(200));
            LED_Set(GREEN_LED, false);
        }
    }
}

void vDisplayTask(void *pvParameters) {
    (void)pvParameters;
    vPrintString("Display Task Started\n");
    
    for (;;) {
        UBaseType_t spotsAvailable;
        GateState_t currentState;
        
        currentState = GateState_Get();
        spotsAvailable = uxSemaphoreGetCount(xParkingSpotsSemaphore);
        vPrintStringAndNumber("Display: Available spots:", spotsAvailable);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int main(void) {
    Hardware_Init();
    vPrintString("\n=== Parking System Initialized ===\n");
    
    xObstacleSemaphore = xSemaphoreCreateBinary();
    xEntrySemaphore = xSemaphoreCreateBinary();
    xExitSemaphore = xSemaphoreCreateBinary();
    xParkingSpotsSemaphore = xSemaphoreCreateCounting(MAX_PARKING_SPOTS, MAX_PARKING_SPOTS);
    xGateStateMutex = xSemaphoreCreateMutex();
    
    vPrintStringAndNumber("Max parking spots:", MAX_PARKING_SPOTS);
    vPrintString("Creating tasks...\n");
    
    xTaskCreate(vSafetyTask,  "Safety",  256, NULL, 4, NULL);
    xTaskCreate(vEntryTask,   "Entry",   256, NULL, 2, NULL);
    xTaskCreate(vExitTask,    "Exit",    256, NULL, 2, NULL);
    xTaskCreate(vDisplayTask, "Display", 256, NULL, 1, NULL);
    
    vPrintString("Starting scheduler...\n");
    vTaskStartScheduler();
    
    for (;;) {}
}
