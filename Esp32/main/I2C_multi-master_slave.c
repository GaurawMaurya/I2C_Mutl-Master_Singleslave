#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "string.h"
#include "driver/gpio.h"

/* Private function prototypes -----------------------------------------------*/
static void uart_init();
static void i2c_init();

static const char *UART_TAG = "UART_Task" ;
static const char *TAG = "Mutex_Tag" ;
static const char *I2C_TAG = "I2C_Task" ;
static const int UART_BUFF = 1024;

//static TaskHandle_t uart_task_handle = NULL;
static SemaphoreHandle_t lcd_mutex;

#define TXD_PIN (GPIO_NUM_4)                                    //Macros defined for GPIO transmitting UART pin
#define RXD_PIN (GPIO_NUM_5)                                    //Macros defined for GPIO Receiving UART pin
#define SDA_PIN (GPIO_NUM_22)
#define SCL_PIN (GPIO_NUM_21)


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
    ESP_LOGI(UART_TAG, "UART parameter configured");

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(UART_TAG, "UART pin configured");

    QueueHandle_t uart_queue;
    esp_err_t err = uart_driver_install(UART_NUM_1, UART_BUFF*2, UART_BUFF*2, 1, &uart_queue, 0);
    if(err !=ESP_OK){
        ESP_LOGE(UART_TAG, "uart driver install failed: %s", esp_err_to_name(err));
        return;
    }

    uart_flush(UART_NUM_1);                                                                     // Clear UART buffer
    ESP_LOGI(UART_TAG, "UART driver installed");
}


static void i2c_init(){
    ESP_LOGI(I2C_TAG, "Configuring I2C");

    i2c_config_t i2c = {
        .mode               = I2C_MODE_MASTER,
        .sda_io_num         = SDA_PIN,
        .scl_io_num         = SCL_PIN,
        .sda_pullup_en      = GPIO_PULLUP_ENABLE,
        .scl_pullup_en      = GPIO_PULLUP_ENABLE,
        .master.clk_speed   = 1000000

    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c));
    esp_err_t err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if(err != ESP_OK){
        ESP_LOGI(I2C_TAG, "I2C driver install failed = %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(I2C_TAG, "I2C Driver install");    
}

/**
 * @brief UART Task - Waits for "LCD_FREE" message and releases LCD mutex
 */
void uart_event_task(void *arg) {
    uint8_t data[UART_BUFF*2];

    while (1) {
        // Wait until UART ISR notifies this task
        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(UART_TAG, "wating for RX data");
        // Read received UART data
        int len = uart_read_bytes(UART_NUM_1, data, UART_BUFF, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the received string

            ESP_LOGI(UART_TAG, "Received: %s", (char *)data);

            // If received message is "LCD_FREE", release LCD mutex
            if (strcmp((char *)data, "LCD_FREE") == 0) {
                ESP_LOGI(UART_TAG, "LCD access granted!");
                xSemaphoreGive(lcd_mutex); // ONLY give mutex here
                ESP_LOGI(TAG, "Lcd mutex released");
            }
        }
    }
}

void uart_send_message(const char *message){
    uart_write_bytes(UART_NUM_1, message, strlen(message));
}

void i2c_lcd_task(void *args)
{
    while (1){
        ESP_LOGI(TAG, "Waiting for I2C access.");
        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "Mutex for Lcd given");
        ESP_LOGI(I2C_TAG, "Using LCD...");

        // ✅ Use uart_send_message(), NOT uart_event_task()
        uart_send_message("LCD_FREE");
        ESP_LOGI(I2C_TAG, "LCD Free, notifying other master...");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    // Create the LCD mutex
    lcd_mutex = xSemaphoreCreateMutex();

    // Start the mutex in a "taken" state so LCD isn't accessed at the start
    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Mutex for Lcd created");

    uart_init();
    i2c_init();

    // Start UART event handling task
    xTaskCreatePinnedToCore(uart_event_task, "uart_event_task", 4096, NULL, 15,NULL, 0);
    xTaskCreatePinnedToCore(i2c_lcd_task, "i2c_lcd_task", 4096, NULL, 12, NULL, 1);
}