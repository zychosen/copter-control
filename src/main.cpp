#include <Arduino.h>
#include <U8g2lib.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include "pid.h"
#include "BluetoothSerial.h"
#include <driver/gpio.h>
#include "u8g2_esp32_hal.h"

#define MOTOR_ENABLE 14
#define PWM_PIN 13
#define DAC_PWM DAC_CHANNEL_1
#define DAC_ANGLE DAC_CHANNEL_2
#define PIN_SDA gpio_num_t(21)
#define PIN_SCL gpio_num_t(22)

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

BluetoothSerial SerialBT;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t task_adc_avg = NULL;
static TaskHandle_t task_bluetooth = NULL;

static int8_t angleInput = -90;
static double Setpoint=80, Output, Kp=0.6, Kd=0.1, Ki=0.6; // 4 0.2 1 // 2.5 0.29 0.45 // 170 26000 0.0625
static double adcVal, beta = 0.2;

PID myPID(&adcVal, &Output, &Setpoint, Kp, Ki, Kd, 1);

static const int len = 10;
static const char cmd[] = "off\0";
static const char kp[] = "kp ";
static const char kd[] = "kd ";
static const char ki[] = "ki ";
static const char b[] = "beta ";
static int8_t angle_dup = -90;

static hw_timer_t *timer = NULL;
static hw_timer_t *timer2 = NULL;
static const uint16_t timer_divider = 80;
static const uint64_t timer_max_count = 1000;

enum { BUF_LEN = 32 };  
static volatile uint16_t buf_0[BUF_LEN];     
static volatile uint16_t buf_1[BUF_LEN];      
static volatile uint16_t* write_to = buf_0;   
static volatile uint16_t* read_from = buf_1;  
static volatile uint8_t buf_overrun = 0; 
static SemaphoreHandle_t sem_done_reading = NULL;

const uint16_t freq = 2000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 8;

void IRAM_ATTR swap() {
  volatile uint16_t* temp_ptr = write_to;
  write_to = read_from;
  read_from = temp_ptr;
}

void IRAM_ATTR onTimer() {
    BaseType_t task_woken = pdFALSE;
    myPID.compute();
    GPIO.out_w1ts = 1 << MOTOR_ENABLE;
    ledcWrite(ledChannel, Output);
    vTaskNotifyGiveFromISR(task_bluetooth, &task_woken);

    if (task_woken) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR onTimer2() {

  static uint16_t idx = 0;
  BaseType_t task_woken = pdFALSE;

  if ((idx < BUF_LEN) && (buf_overrun == 0)) {
    write_to[idx] = adc1_get_raw(ADC1_CHANNEL_0);
    idx++;
  }

  if (idx >= BUF_LEN) {
    if (xSemaphoreTakeFromISR(sem_done_reading, &task_woken) == pdFALSE) {
      buf_overrun = 1;
    }

    if (buf_overrun == 0) {
      idx = 0;
      swap();
      vTaskNotifyGiveFromISR(task_adc_avg, &task_woken);
    }
  }

    if (task_woken) {
      portYIELD_FROM_ISR();
    }
}

void avgADC(void *params) {
    float avg;
    int x;
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        avg = 0.0;
        for (int i = 0; i < BUF_LEN; i++) {
            avg += (float)read_from[i];
        }
        avg /= BUF_LEN;
        x = map(avg, 80, 640, 0, 255);
        dac_output_voltage(DAC_ANGLE, x);
        portENTER_CRITICAL(&spinlock);
        adcVal = avg;
        buf_overrun = 0;
        xSemaphoreGive(sem_done_reading);
        portEXIT_CRITICAL(&spinlock);
    }
}

void listen(void *params){
  char c;
  char buf[len];
  int8_t angle;                          
  uint8_t idx = 0;                       
  memset(buf, 0, len);               
    while (1) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      if (SerialBT.available() > 0) {
        c = SerialBT.read();

      if (c == '\n') {               
        buf[idx] = '\0'; 
        if (memcmp(buf, cmd, 4) == 0) {
          angle = -90;
          goto x;
        } else if (memcmp(buf, kp, 3) == 0) {
            portENTER_CRITICAL(&spinlock);
            Kp = atof(buf + 3);
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } else if (memcmp(buf, kd, 3) == 0) {
            portENTER_CRITICAL(&spinlock);
            Kd = atof(buf + 3);
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } else if (memcmp(buf, ki, 3) == 0) {
            portENTER_CRITICAL(&spinlock);
            Ki = atof(buf + 3);
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } else if (memcmp(buf, b, 5) == 0) {
            portENTER_CRITICAL(&spinlock);
            beta = atof(buf + 5);
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } 
          else {
            angle = atoi(buf);
            angle_dup = angle;
        }

        x : angle_dup = angle;
        portENTER_CRITICAL(&spinlock);
        Setpoint = 430 + angle*3.88;
        dac_output_voltage(DAC_PWM, map(Setpoint, 80, 640, 0, 255));
        angleInput = angle;
        portEXIT_CRITICAL(&spinlock);
        memset(buf, 0, len);
        idx = 0;                   
      }
      
      if (idx < len - 1 && (isalnum(c) || c == '-' || c == ' ' || c == '.')) { 
        buf[idx] = c;
        idx++;
      }
    }
  }
}

void OLED(void *ignore) {
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda  = PIN_SDA;
  u8g2_esp32_hal.scl  = PIN_SCL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);
  u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  char kp[10], ki[10], kd[10], Beta[10];
  memset(kp, 0, 10); memset(ki, 0, 10); memset(kd, 0, 10); memset(kd, 0, 10);

  while(1) {
    portENTER_CRITICAL(&spinlock);
    int16_t adc = adcVal;
    portEXIT_CRITICAL(&spinlock);
    portENTER_CRITICAL(&spinlock);
    sprintf(kp, "%0.3f", myPID.GetKp()); 
    sprintf(ki, "%0.3f", myPID.GetKi());
    sprintf(kd, "%0.3f", myPID.GetKd());
    sprintf(Beta, "%0.2f", myPID.GetBeta());
    portEXIT_CRITICAL(&spinlock);
    
    adc = 0.257*(adc - 430);
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_t0_11b_me);
    u8g2_DrawStr(&u8g2, 13, 17 , "Copter Parameters");
    u8g2_DrawHLine(&u8g2, 2, 20, 128);
    u8g2_DrawStr(&u8g2, 2, 32 ,"DC:");
    portENTER_CRITICAL(&spinlock);
    u8g2_DrawStr(&u8g2, 25, 32, u8x8_utoa(Output/255*100));
    portEXIT_CRITICAL(&spinlock);
    u8g2_DrawUTF8(&u8g2, 45, 32, "%");
    u8g2_DrawStr(&u8g2, 2, 43, "SP:");
    if (angleInput < 0) {
      u8g2_DrawUTF8(&u8g2, 25, 43, "-");
      portENTER_CRITICAL(&spinlock);
      u8g2_DrawStr(&u8g2, 35, 43, u8x8_utoa(abs(angleInput)));
      portEXIT_CRITICAL(&spinlock);
      u8g2_DrawUTF8(&u8g2, 50, 43, "°");
    } else {
       portENTER_CRITICAL(&spinlock);
       u8g2_DrawStr(&u8g2, 25, 43, u8x8_utoa(abs(angleInput)));
       portEXIT_CRITICAL(&spinlock);
       u8g2_DrawUTF8(&u8g2, 40, 43, "°");
    }
    u8g2_DrawStr(&u8g2, 2, 54, "PV:");
    if (adc < 0) {
      u8g2_DrawUTF8(&u8g2, 25, 54, "-");
      u8g2_DrawStr(&u8g2, 35, 54, u8x8_utoa(abs(adc)));
      u8g2_DrawUTF8(&u8g2, 50, 54, "°");
    } else {
      u8g2_DrawStr(&u8g2, 25, 54, u8x8_utoa(abs(adc)));
      u8g2_DrawUTF8(&u8g2, 40, 54, "°");
    }

    u8g2_DrawUTF8(&u8g2, 65, 64 , "B :");
    u8g2_DrawStr(&u8g2, 88, 64 , Beta);

    u8g2_DrawStr(&u8g2, 65, 32 , "Kp:");
    u8g2_DrawStr(&u8g2, 88, 32, kp);
    u8g2_DrawStr(&u8g2, 65, 43 , "Ki:");
    u8g2_DrawStr(&u8g2, 88, 43, ki);
    u8g2_DrawStr(&u8g2, 65, 54 , "Kd:");
    u8g2_DrawStr(&u8g2, 88, 54, kd);
    u8g2_SendBuffer(&u8g2);
    vTaskDelay(200/portTICK_PERIOD_MS);
  } 
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("1 DoF copter");

  adc1_config_width(ADC_WIDTH_10Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  dac_output_enable(DAC_PWM);
  dac_output_enable(DAC_ANGLE);
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(PWM_PIN, ledChannel);

  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  sem_done_reading = xSemaphoreCreateBinary();

  if (sem_done_reading == NULL) {
        Serial.println("Could not create one or more semaphores");
        ESP.restart();
    }

  xSemaphoreGive(sem_done_reading);
  myPID.limitOutput(0, 255);

  xTaskCreatePinnedToCore(avgADC,
                          "ADC avg",
                          1024,
                          NULL,
                          1,
                          &task_adc_avg,
                          1);

  xTaskCreatePinnedToCore(listen,
                          "Bluetooth",
                          1024,
                          NULL,
                          1,
                          &task_bluetooth,
                          0);

  xTaskCreatePinnedToCore(OLED,
                          "OLED Display",
                          2048,
                          NULL,
                          1,
                          NULL,
                          0);
  
  timer = timerBegin(0, timer_divider, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timer_max_count, true);
  timerAlarmEnable(timer);

  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 30, true);
  timerAlarmEnable(timer2);
  vTaskDelete(NULL);
}

void loop() {

}