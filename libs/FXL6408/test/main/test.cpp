#include "sdkconfig.h"

#include "esp_log.h"

#include "FXL6408/fxl6408.hpp"
#include "freertos/task.h"

static const char* TAG = "test.cpp";

#define CONFIG_SDA_GPIO GPIO_NUM_27
#define CONFIG_SCL_GPIO GPIO_NUM_26
#define CONFIG_WP_GPIO GPIO_NUM_12

#define EXPANDER_RST_GPIO GPIO_NUM_15
#define EXPANDER_INT_GPIO GPIO_NUM_39

TaskHandle_t thread = nullptr;
TaskHandle_t status_thread = nullptr;

void fxl6408_test_communication(GpioExpander::GpioExpander *test);
void fxl6408_test_reset(GpioExpander::GpioExpander *test);

void fxl6408_test_communication(GpioExpander::GpioExpander *test)
{
    printf("\r\n**********[FXL6408] COMMUNICATION TEST BEGIN**********\r\n");

	esp_err_t err = ESP_OK;
    uint8_t data = 0;

    err = test->fxl6408_read_ctrl(&data);
    if (err != ESP_OK)
        printf("\r\n**********[FXL6408] COMMUNICATION TEST FAIL**********\r\n");
    else
    {
        printf("\r\n**********[FXL6408] COMMUNICATION TEST OK**********\r\n");
        printf("data = %u\r\n", data);
    }
}

void fxl6408_test_reset(GpioExpander::GpioExpander *test)
{
    printf("\r\n**********[FXL6408] RESET TEST BEGIN**********\r\n");

	esp_err_t err = ESP_OK;
    uint8_t data = 0;

    err = test->fxl6408_reset();
    if (err != ESP_OK)
        printf("\r\n**********[FXL6408] RESET TEST FAIL**********\r\n");
    else
    {
        test->fxl6408_read_ctrl(&data);
        if (data != 0xA2)
            printf("\r\n**********[FXL6408] RESET TEST FAIL**********\r\n");
        else
            printf("\r\n**********[FXL6408] RESET TEST OK**********\r\n");
        printf("data = %u\r\n", data);
    }
}

// STM66_GPIO5 INT
// STM66_GPIO6 PS_HOLD

void stm66_int_task(void *arg)
{
    for (;;)
    {
        uint32_t event = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
        printf("stm66_int_task was notified\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void int_status_task(void *arg)
{
    uint8_t status = 0;

    GpioExpander::GpioExpander *expander = static_cast<GpioExpander::GpioExpander *>(arg);

    for (;;)
    {
        expander->fxl6408_read_it_status(&status);
        printf("status %u\r\n", status);

        expander->fxl6408_read_input_status(&status);
        printf("status %u\r\n", status);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    I2C::I2CMaster *i2c = new I2C::I2CMaster();
	
    i2c_port_t port = I2C_NUM_0;
    int sda = CONFIG_SDA_GPIO;
    int scl = CONFIG_SCL_GPIO;
    int wp = CONFIG_WP_GPIO;
    int rst = EXPANDER_RST_GPIO;
    int it = EXPANDER_INT_GPIO;

    ESP_ERROR_CHECK(i2c->init(port, 400000, sda, scl));

    GpioExpander::GpioExpander *dev = new GpioExpander::GpioExpander();

	ESP_ERROR_CHECK(dev->init(i2c, rst, it, 0x01));

    fxl6408_test_communication(dev);
    fxl6408_test_reset(dev);
    
    // STM66_INT
    dev->fxl6408_set_io_dir(FXL6408_GPIO_5, FXL6408_GPIO_MODE_INPUT);
    dev->fxl6408_set_it_mask(FXL6408_GPIO_5, FXL6408_GPIO_NO_MASK);
    dev->fxl6408_set_input_state(FXL6408_GPIO_5, FXL6408_GPIO_INPUT_DEFAULT_HIGH);

    dev->fxl6408_set_task(&thread, GpioExpander::GPIO_EXPANDER_IO_5);

    uint8_t status = 0;
    dev->fxl6408_read_it_status(&status);

    auto ok = xTaskCreate(stm66_int_task, "stm66_int_task", 1024,
                        NULL, 10, &thread);
}
