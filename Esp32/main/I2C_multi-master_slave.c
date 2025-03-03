#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

/* Private function prototypes -----------------------------------------------*/
static void uart_init();
static void i2c_init();

static const char *UART_TAG = "UART_Task" ;
static const char *I2C_TAG = "I2C_Task" ;

#define TXD_PIN (GPIO_NUM_4)                                    //Macros defined for GPIO transmitting UART pin
#define RXD_PIN (GPIO_NUM_5)                                    //Macros defined for GPIO Receiving UART pin
#define SDA_PIN (GPIO_NUM)
#define SCL_PIN (GPIO_NUM)

static void uart_init(){
    ESP_LOGI(UART_TAG, "configuring UART");

    uart_config_t uart_config = {
        .baud_rate  =   115200,
        .data_bits  =   UART_DATA_8_BITS,
        .parity     =   UART_PARITY_DISABLE,
        .stop_bits  =   UART_STOP_BITS_1,
        .flow_ctrl  =   UART_HW_FLOWCTRL_DISABLE 
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
}

static void i2c_init(){

}

void app_main(void)
{
   uart_init();

   i2c_init();


}