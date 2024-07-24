// Libraries
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "Button.h"
#include "esp_timer.h"
// --------------------------------------------------
// Variables
#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define PIN_NUM_SWITCH 4
#define PIN_NETLIGHT 37

#define NETLIGHT_REPORT_TIMEOUT 60000000 //us

typedef enum
{
    OFF,
    NOT_REGISTERED,
    REGISTERED,
    GPRS_CONNECTED,
    UNKNOWN = 0xff,
} NetlightStatus_t;

typedef struct
{
    NetlightStatus_t status;
    unsigned long positivePulseWidth;
    unsigned long negativePulseWidth;
    unsigned long lastNegativePulseStartTime;
} R800CNetlight_t;

static const char *TAG = "main";
static esp_timer_handle_t netlightTimer;
static QueueHandle_t xQueueR800CNetlight;

void power_on_modem()
{
    printf("Power on the modem\r\n");
    gpio_set_level(GPIO_NUM_25, true);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_25, false);
}

const char* printR800CNetlightStatus(NetlightStatus_t status)
{
    switch(status)
    {
        case OFF:
            return "Powered off";
        break;

        case NOT_REGISTERED:
            return "Not registered the network";
        break;

        case REGISTERED:
            return "Registered to the network";
        break;

        case GPRS_CONNECTED:
            return "GPRS communication is established";
        break;

        case UNKNOWN:
            return "UNKNOWN";
        break;
    }
    return "ERROR";
}

static void IRAM_ATTR calcPulsewidth()
{
    static unsigned long positivePulseStartTime = 0, negativePulseStartTime = 0;
    static NetlightStatus_t state = OFF, prev_state = UNKNOWN;
    BaseType_t xHigherPriorityTaskWoken;
    R800CNetlight_t m_R800CNetlight;
    
    static int64_t lastReport = 0;
    int64_t now = micros();

    /* We have not woken a task at the start of the ISR. */
    xHigherPriorityTaskWoken = pdFALSE;

    if (gpio_get_level((gpio_num_t)PIN_NETLIGHT) == HIGH) // If the change was a RISING edge
    {
        positivePulseStartTime = now; // Store the start time (in microseconds)
        m_R800CNetlight.negativePulseWidth = (now - negativePulseStartTime) / 1000; // pulse in ms
    }
    else // If the change was a FALLING edge
    {
        m_R800CNetlight.positivePulseWidth = (now - positivePulseStartTime) / 1000; // pulse in ms
        negativePulseStartTime = now;
        m_R800CNetlight.lastNegativePulseStartTime = now;

        if(esp_timer_is_active(netlightTimer))
        {
            esp_timer_stop(netlightTimer);
        }
        esp_timer_start_once(netlightTimer, 3300000);
    }

    if (m_R800CNetlight.positivePulseWidth > 0)
    {
        // 300ms +- 10%
        if (m_R800CNetlight.negativePulseWidth < 330 && m_R800CNetlight.negativePulseWidth > 270)
            state = GPRS_CONNECTED;

        // 800ms +- 10%
        else if (m_R800CNetlight.negativePulseWidth < 880 && m_R800CNetlight.negativePulseWidth > 720)
            state = NOT_REGISTERED;

        // 3000ms +- 10%
        else if (m_R800CNetlight.negativePulseWidth < 3300 && m_R800CNetlight.negativePulseWidth > 2700)
            state = REGISTERED;
    }
    else
    {
        state = OFF;
        m_R800CNetlight.negativePulseWidth = 0;
    }

    if (state != prev_state)
    {
        prev_state = state;
        lastReport = now;
        m_R800CNetlight.status = state;
        xQueueSendFromISR(xQueueR800CNetlight, &m_R800CNetlight, NULL);
    }
    else
    {
        if(now - lastReport > NETLIGHT_REPORT_TIMEOUT)
        {
            lastReport = now;
            m_R800CNetlight.status = state;
            xQueueSendFromISR(xQueueR800CNetlight, &m_R800CNetlight, NULL);
        }
    }
}

static void netlightTimerCallback(void *arg)
{
    R800CNetlight_t R800CNetlight;
    R800CNetlight.status = OFF;
    R800CNetlight.lastNegativePulseStartTime = 0;
    R800CNetlight.negativePulseWidth = 0;
    R800CNetlight.positivePulseWidth = 0;
    xQueueSend(xQueueR800CNetlight, &R800CNetlight, 0);
}

void statusTask(void *pvParameters)
{
    int16_t count = 0;
    int64_t now = 0;
    R800CNetlight_t qElementNetlight;
    
    const esp_timer_create_args_t netlightTimerArgs = {
        .callback = &netlightTimerCallback,
        .name = "netlightTimer"};

    ESP_ERROR_CHECK(esp_timer_create(&netlightTimerArgs, &netlightTimer));
    gpio_config_t io_conf = {}; // zero-initialize the config structure.

    io_conf.intr_type = GPIO_INTR_DISABLE;                                                                                                                // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                                                                                                      // set as output mode
    io_conf.pin_bit_mask = GPIO_PIN_SEL(GPIO_OUTPUT_POWER_ON) | GPIO_PIN_SEL(GPIO_OUTPUT_PWRKEY) | GPIO_PIN_SEL(GPIO_NUM_25) | GPIO_PIN_SEL(GPIO_NUM_33); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                                                                                         // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                                                                             // disable pull-up mode

    gpio_config(&io_conf); // configure GPIO with the given settings

    gpio_set_level(GPIO_OUTPUT_POWER_ON, true);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, false);
    gpio_set_level(GPIO_NUM_25, false);

    xQueueR800CNetlight = xQueueCreate(3, sizeof(R800CNetlight_t));
    if(xQueueR800CNetlight == NULL)
    {
        printf("\t\t\t ERROR!\r\n");
        while(1);
    }

    pinMode(PIN_NETLIGHT, INPUT);                                                 // Set the input pin
    attachInterrupt(digitalPinToInterrupt(PIN_NETLIGHT), calcPulsewidth, CHANGE); // Run the calcPulsewidth function on signal CHANGE

    if(digitalRead(PIN_NETLIGHT) == 0)
    {
        esp_timer_start_once(netlightTimer, 3300000);
    }
    
    power_on_modem();
    
    while (1)
    {
        if(xQueueReceive(xQueueR800CNetlight, &qElementNetlight, portMAX_DELAY))
        {
            printf("\t\t %s | positive %ld ms | negative %ld ms\r\n", 
                printR800CNetlightStatus(qElementNetlight.status),
                qElementNetlight.positivePulseWidth, qElementNetlight.negativePulseWidth);
        }
    }
}

void button_handler(button_queue_t param)
{
    switch (param.buttonState)
    {
    case BTN_PRESSED:
        ESP_LOGD("BTN", "pressed");
        power_on_modem();
        break;

    case BTN_RELEASED:

        if (param.buttonPrev == BTN_PRESSED)
        {
        }
        break;

    case BTN_HOLD:
        ESP_LOGI("BTN", "hold");
        break;
    }
}

void setup()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);
   
    button_init_t button = {
        .buttonEventHandler = button_handler,
        .pin_bit_mask = (1ULL << PIN_NUM_SWITCH),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE};

    buttonInit(&button);

    xTaskCreatePinnedToCore(statusTask, "statusTask", 4096, NULL, 5, NULL, 0);
}

// --------------------------------------------------
// --------------------------------------------------
void loop()
{
    if (Serial.available())
    {
        Serial1.write(Serial.read());
    }

    // computer to R800C
    if (Serial1.available())
    {
        Serial.write(Serial1.read());
    }
    vTaskDelay(pdMS_TO_TICKS(10));
}
