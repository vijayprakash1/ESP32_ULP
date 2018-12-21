#include <Arduino.h>


#include "esp32/ulp.h"
#include "driver/rtc_io.h"
MOSI

/*#include <Wire.h>
//#include "apds9301.h"

#define IMUinterruptPin     12
#define LIGHTinterruptPin   13


volatile bool KV_LIGHT_INT_HIGH_STATUS = false;
volatile bool KV_IMU_INT_HIGH_STATUS = false;
void  kv_imu_interrupt_func() {
  KV_IMU_INT_HIGH_STATUS = true;
}
void  kv_light_interrupt_func() {
  KV_LIGHT_INT_HIGH_STATUS = true;
}




#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))

void ULP_BLINK_RUN(uint32_t us);



LuxI2C_APDS9301 light = LuxI2C_APDS9301(0x39);



void setup_light(){
    Wire.begin();


    light.powerOn();
    delay(2);

    if(light.isPowerOn()){
        Serial.println("LIGHT SENSOR IS ON");
    }
    else{
        Serial.println("LIGHT SENSOR IS OFF");
    }


    light.setADCGain(LuxI2C_APDS9301::ADCGain::high_gain);

    light.setLowThreshold(0);
    light.setHighThreshold(900);
    light.setOutputInterruptOn();


    light.setIntegrationTime(LuxI2C_APDS9301::low_time);
    light.setGenerateInterruptPeriod(2);
    light.clearInterrupt();
}
*/
void ULP_READ_INT() {
    RTC_SLOW_MEM[12] = 0;
    ulp_set_wakeup_period(0, 100000L);
    const ulp_insn_t ulp_blink[] = {
        M_LABEL(1), // BEGIN
        // I_RD_REG(RTC_GPIO_IN_REG, 14, 14), // BUTTON (PIN 36)
        I_RD_REG(RTC_GPIO_IN_REG, 29, 29), // GPIO_12
        M_BL(3,1), // if 0 go to label:2
        M_BX(4),   // else go to label:3
        
        M_LABEL(2),
        I_RD_REG(RTC_GPIO_IN_REG, 28, 28), // GPIO_13
       // I_RD_REG(RTC_GPIO_IN_REG, 24, 25),
        M_BL(6,1), // if 0 go to label:2
        M_BX(5),   // else go to label:3
       
        M_LABEL(3), //  LOW
        I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 0),   // GPIO2
        M_BX(2),
        // I_HALT(),

        M_LABEL(4), //  HIGH  
        I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 1),
        M_BX(2),  
        // I_HALT(),
        M_LABEL(5), //  HIGH
        I_WR_REG(RTC_GPIO_OUT_REG, 24, 24, 1),     // GPIO13
        M_BX(1),
        
        M_LABEL(6), //  HIGH
        I_WR_REG(RTC_GPIO_OUT_REG, 24, 24, 0),
        M_BX(1)
        // I_WAKE()

        // I_MOVI(R3, 12),                        // #12 -> R3
        // I_LD(R0, R3, 0),                       // R0 = RTC_SLOW_MEM[R3(#12)]
        // M_BL(1, 1),                            // GOTO M_LABEL(1) IF R0 < 1
        // I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 1), // RTC_GPIO12 = 1
        // I_SUBI(R0, R0, 1),                     // R0 = R0 - 1, R0 = 1, R0 = 0
        // I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        // M_BX(2),                               // GOTO M_LABEL(2)
        // M_LABEL(1),                            // M_LABEL(1)
        // I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 0), // RTC_GPIO12 = 0
        // // I_WAKE(),
        // I_ADDI(R0, R0, 1),                     // R0 = R0 + 1, R0 = 0, R0 = 1
        // I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        // M_LABEL(2),                            // M_LABEL(2)
        // I_HALT()                               // HALT COPROCESSOR
    };
    

    rtc_gpio_init(GPIO_NUM_2);
    rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_2, 0);


    rtc_gpio_init(GPIO_NUM_4);
    rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_4, 0);

     rtc_gpio_init(GPIO_NUM_12);
     rtc_gpio_set_direction(GPIO_NUM_12, RTC_GPIO_MODE_INPUT_ONLY);
     uint32_t l = rtc_gpio_get_level(GPIO_NUM_12);
     Serial.printf("RTC GPIO 12 LEVEL : %d\n", l);
     
     rtc_gpio_init(GPIO_NUM_13);
     rtc_gpio_set_direction(GPIO_NUM_13, RTC_GPIO_MODE_INPUT_ONLY);
     uint32_t l2 = rtc_gpio_get_level(GPIO_NUM_13);
     Serial.printf("RTC GPIO 13 LEVEL : %d\n", l2);
    //rtc_gpio_init(GPIO_NUM_12);
    //rtc_gpio_set_direction(GPIO_NUM_12, RTC_GPIO_MODE_INPUT_ONLY);
   // Serial.printf("RTC GPIO 12 LEVEL : %d\n", rtc_gpio_get_level(GPIO_NUM_12));
    
    
    // rtc_gpio_init(GPIO_NUM_36);
    // rtc_gpio_set_direction(GPIO_NUM_36, RTC_GPIO_MODE_INPUT_ONLY);
    // uint32_t l = rtc_gpio_get_level(GPIO_NUM_36);
    // Serial.printf("RTC GPIO 36 LEVEL : %d\n", l);


    size_t size = sizeof(ulp_blink) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_blink, &size);
    ulp_run(0);
}

void READ_GPIO() { 
    RTC_SLOW_MEM[12] = 0;
    ulp_set_wakeup_period(0, 1000000L);
    const ulp_insn_t ulp_blink[] = {
        I_RD_REG(RTC_GPIO_IN_REG, 24, 25),

        M_BL(1, 1),                            // GOTO M_LABEL(1) IF R0 < 1
        I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 1), // RTC_GPIO12 = 1
        I_SUBI(R0, R0, 1),                     // R0 = R0 - 1, R0 = 1, R0 = 0
        I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        M_BX(2),                               // GOTO M_LABEL(2)
        M_LABEL(1),                            // M_LABEL(1)
        I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 0), // RTC_GPIO12 = 0
        I_ADDI(R0, R0, 1),                     // R0 = R0 + 1, R0 = 0, R0 = 1
        I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        M_LABEL(2),                            // M_LABEL(2)
        I_HALT()                               // HALT COPROCESSOR
    };
    
    rtc_gpio_init(GPIO_NUM_2);
    rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_2, 0);

    
    rtc_gpio_init(GPIO_NUM_4);
    rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_4, 0);

    size_t size = sizeof(ulp_blink) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_blink, &size);
    ulp_run(0);
}

void ULP_BLINK_13() {
    RTC_SLOW_MEM[12] = 0;
    ulp_set_wakeup_period(0, 1000000L);
    const ulp_insn_t ulp_blink[] = {
        I_MOVI(R3, 12),                        // #12 -> R3
        I_LD(R0, R3, 0),                       // R0 = RTC_SLOW_MEM[R3(#12)]
        M_BL(1, 1),                            // GOTO M_LABEL(1) IF R0 < 1
        I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 1), // RTC_GPIO12 = 1
        I_SUBI(R0, R0, 1),                     // R0 = R0 - 1, R0 = 1, R0 = 0
        I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        M_BX(2),                               // GOTO M_LABEL(2)
        M_LABEL(1),                            // M_LABEL(1)
        I_WR_REG(RTC_GPIO_OUT_REG, 26, 27, 0), // RTC_GPIO12 = 0
        // I_WAKE(),
        I_ADDI(R0, R0, 1),                     // R0 = R0 + 1, R0 = 0, R0 = 1
        I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        M_LABEL(2),                            // M_LABEL(2)
        I_HALT()                               // HALT COPROCESSOR
    };
    
    rtc_gpio_init(GPIO_NUM_2);
    rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_2, 0);


    size_t size = sizeof(ulp_blink) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_blink, &size);
    ulp_run(0);
}
void ULP_BLINK_RUN(uint32_t us) {
    RTC_SLOW_MEM[12] = 0;
    ulp_set_wakeup_period(0, us);
    const ulp_insn_t ulp_blink[] = {
        I_MOVI(R3, 12),                        // #12 -> R3
        I_LD(R0, R3, 0),                       // R0 = RTC_SLOW_MEM[R3(#12)]
        M_BL(1, 1),                            // GOTO M_LABEL(1) IF R0 < 1
        I_WR_REG(RTC_GPIO_OUT_REG, 24, 27, 0b0111), // RTC_GPIO12 = 1
        I_SUBI(R0, R0, 1),                     // R0 = R0 - 1, R0 = 1, R0 = 0
        I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        M_BX(2),                               // GOTO M_LABEL(2)
        M_LABEL(1),                            // M_LABEL(1)
        I_WR_REG(RTC_GPIO_OUT_REG, 24, 25, 0), // RTC_GPIO12 = 0
        // I_WAKE(),
        I_ADDI(R0, R0, 1),                     // R0 = R0 + 1, R0 = 0, R0 = 1
        I_ST(R0, R3, 0),                       // RTC_SLOW_MEM[R3(#12)] = R0
        M_LABEL(2),                            // M_LABEL(2)
        I_HALT()                               // HALT COPROCESSOR
    };
    
    rtc_gpio_init(GPIO_NUM_2);
    rtc_gpio_set_direction(GPIO_NUM_2, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_2, 0);

    
    rtc_gpio_init(GPIO_NUM_4);
    rtc_gpio_set_direction(GPIO_NUM_4, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_NUM_4, 0);

    size_t size = sizeof(ulp_blink) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, ulp_blink, &size);
    ulp_run(0);
}

void setup() {
    Serial.begin(115200);
    delay(1000);


    // uint64_t BM = 0;
    // BIT_SET(BM, 36);
    // esp_sleep_enable_ext1_wakeup(BM, ESP_EXT1_WAKEUP_ALL_LOW);



    //setup_light();
    // attachInterrupt(digitalPinToInterrupt(LIGHTinterruptPin), kv_light_interrupt_func, FALLING);
    
    
   // READ_GPIO();
   // ULP_BLINK_RUN(1000000);
    ULP_READ_INT();
    //ULP_BLINK_13();
   // ULP_BLINK_RUN(1000000);


    return;
    Serial.println("SLEEPING NOW");
    delay(1000);
    esp_sleep_enable_ulp_wakeup();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_deep_sleep_start();
}


void loop()
{
    /*if(KV_LIGHT_INT_HIGH_STATUS){
        KV_LIGHT_INT_HIGH_STATUS = false;
        light.clearInterrupt();
        Serial.printf("LIGHT INT\n");
    }

    delay(100);*/
}

