// Libraries
#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "Button.h"
// --------------------------------------------------
// Variables
#define GPIO_PIN_SEL(pin) (1ULL << pin)
#define TXD_PIN ((gpio_num_t)GPIO_NUM_14)
#define RXD_PIN ((gpio_num_t)GPIO_NUM_34)
#define GPIO_OUTPUT_PWRKEY ((gpio_num_t)GPIO_NUM_16)
#define GPIO_OUTPUT_POWER_ON ((gpio_num_t)GPIO_NUM_13)
#define PIN_NUM_SWITCH 4
#define PIN_NETLIGHT    37

const char *TAG="main";

void power_on_modem()
{
    printf("Power on the modem\r\n");
    gpio_set_level(GPIO_NUM_25, true);
    // gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    gpio_set_level(GPIO_OUTPUT_PWRKEY, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_NUM_25, false);
}

void button_handler (button_queue_t param)
{

    switch(param.buttonState)
    {
        case BTN_PRESSED:
            ESP_LOGD("BTN", "pressed");
            power_on_modem();
        break;

        case BTN_RELEASED:

            if(param.buttonPrev == BTN_PRESSED)
            {
            }
        break;

        case BTN_HOLD:        
            ESP_LOGI("BTN", "hold");
        break;
    }
    
}
volatile unsigned long isrPositiveWidth = 0, isrNegativeWidth = 0, isrLastPulseStartNegativeTime = 0;     // Define Interrupt Service Routine (ISR) pulsewidth
unsigned long positivePulseWidth, negativePulseWidth, lastPulseStartNegativeTime; 

void calcPulsewidth()                    // Interrupt Service Routine (ISR)
{
  static unsigned long pulseStartPositiveTime, pulseStartNegativeTime;   // Start time variable
  int64_t now = micros();
  
  if (digitalRead(PIN_NETLIGHT) == HIGH)    // If the change was a RISING edge
  {
    pulseStartPositiveTime = now;           // Store the start time (in microseconds)
    isrNegativeWidth = now - pulseStartNegativeTime;
  }
  else                                   // If the change was a FALLING edge
  {        
    isrPositiveWidth = now - pulseStartPositiveTime;    // Calculate the pulsewidth
    pulseStartNegativeTime = now;
    isrLastPulseStartNegativeTime = now;
  }
}
typedef enum
{
    OFF,
    NOT_REGISTERED,
    REGISTERED,
    GPRS_CONNECTED,
    UNKNOWN = 0xff,
} NetStatus_t;

void statusTask(void* pvParameters)
{
    NetStatus_t state = OFF, prev_state = UNKNOWN;
    int16_t count = 0;
    int64_t now = 0;
    while(1)
    {
        noInterrupts();
        positivePulseWidth = isrPositiveWidth;
        negativePulseWidth = isrNegativeWidth;
        lastPulseStartNegativeTime = isrLastPulseStartNegativeTime;
        interrupts();
        now = micros();
        if((now - lastPulseStartNegativeTime)/1000 > 3300)
        {
            positivePulseWidth = 0;
            negativePulseWidth = 0;
        } 

        if(positivePulseWidth/1000 > 0)
        {
            //300ms +- 10%
            if(negativePulseWidth/1000 < 330 && negativePulseWidth/1000 > 270)
                state = GPRS_CONNECTED;

            //800ms +- 10%
            else if (negativePulseWidth/1000 < 880 && negativePulseWidth/1000 > 720)
                state = NOT_REGISTERED;
            
            //3000ms +- 10%
            else if(negativePulseWidth/1000 < 3300 && negativePulseWidth/1000 > 2700)
                state = REGISTERED;
        }
        else
            state = OFF;

        
        if(state != prev_state)
        {
            prev_state = state;
            printf("\t\t positive=%ld | negative=%ld\r\n", positivePulseWidth/1000, negativePulseWidth/1000);
            switch(state)
            {
                case OFF:
                    printf("\t\t R800C OFF\r\n");
                break;

                case REGISTERED:
                    printf("\t\t Registered to the network\r\n");
                break;

                case NOT_REGISTERED:
                    printf("\t\t Not registered the network\r\n");
                break;

                case GPRS_CONNECTED:
                    printf("\t\t GPRS communication is established\r\n");
                break;
                
                default:
                    printf("\t\t UNKNOWN STATE\r\n");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
        if (count++ > 1000)
        {
            count = 0;
            printf("\t\t positive=%ld | negative=%ld\r\n", positivePulseWidth/1000, negativePulseWidth/1000);
        }
    }
}

void setup()
{
    esp_log_level_set("*", ESP_LOG_INFO);
    Serial.begin(115200);
    Serial1.begin(115200,SERIAL_8N1,RXD_PIN,TXD_PIN);
    
    pinMode(PIN_NETLIGHT, INPUT);              // Set the input pin
    attachInterrupt(digitalPinToInterrupt(PIN_NETLIGHT), calcPulsewidth, CHANGE);   // Run the calcPulsewidth function on signal CHANGE

    button_init_t button = {
        .buttonEventHandler = button_handler,
        .pin_bit_mask = (1ULL<<PIN_NUM_SWITCH),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };

    buttonInit(&button);    

    gpio_config_t io_conf = {}; // zero-initialize the config structure.

    io_conf.intr_type = GPIO_INTR_DISABLE;                                                        // disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT;                                                              // set as output mode
    io_conf.pin_bit_mask = GPIO_PIN_SEL(GPIO_OUTPUT_POWER_ON) | GPIO_PIN_SEL(GPIO_OUTPUT_PWRKEY) | GPIO_PIN_SEL(GPIO_NUM_25) | GPIO_PIN_SEL(GPIO_NUM_33); // bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;                                                 // disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;                                                     // disable pull-up mode

    gpio_config(&io_conf); // configure GPIO with the given settings

    gpio_set_level(GPIO_OUTPUT_POWER_ON, true);
    gpio_set_level(GPIO_OUTPUT_PWRKEY, false);
    gpio_set_level(GPIO_NUM_25, false);
    power_on_modem();

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
