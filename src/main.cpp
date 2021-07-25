/**
 * 1-DoF Copter control
 * 
 * Creating an RTOS to control a 1-DoF copter using the ESP32
 * 
 * Date: July 25, 2021
 * Authors: zychosen, nitishbhat09, ShreyasRkk
 * License: 0BSD
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include "pid.h"
#include "BluetoothSerial.h"
#include <driver/gpio.h>
#include "u8g2_esp32_hal.h"

/* Pin definitions */
#define MOTOR_ENABLE 14
#define PWM_PIN 13
#define DAC_SP DAC_CHANNEL_1            // DAC output gives setpoint
#define DAC_ANGLE DAC_CHANNEL_2         // DAC output gives current angle
#define PIN_SDA gpio_num_t(21)
#define PIN_SCL gpio_num_t(22)

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

BluetoothSerial SerialBT;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t task_adc_avg = NULL;

static int8_t angleInput = -90;

/* Variables for PID computation */
static float Setpoint=80, Output, Kp=0.8, Kd=0.08, Ki=0.45;  // 4 0.2 1 // 2.5 0.29 0.45 // 170 26000 0.0625
static float adcVal, beta = 0.55;

/* PID object initialization */
PID myPID(&adcVal, &Output, &Setpoint, Kp, Ki, Kd, 1);

/* Bluetooth serial commands */
static const int len = 10;
static const char cmd[] = "off\0";
static const char kp[] = "kp ";
static const char kd[] = "kd ";
static const char ki[] = "ki ";
static const char b[] = "beta ";

static int8_t angle_dup = -90;

/* Initializing hardware timer with 20KHz frequency i.e. 50 counts at 1000000 counts/sec */
static hw_timer_t *timer = NULL;
static const uint16_t timer_divider = 80;
static const uint64_t timer_max_count = 50;

/* Variables for double buffer read/write */
enum { BUF_LEN = 16 };  
static volatile uint16_t buf_0[BUF_LEN];     
static volatile uint16_t buf_1[BUF_LEN];      
static volatile uint16_t* write_to = buf_0;   
static volatile uint16_t* read_from = buf_1;  
static volatile uint8_t buf_overrun = 0; 
static SemaphoreHandle_t sem_done_reading = NULL;

/* PWM channel setup with frequency 2Khz */
const uint16_t freq = 2000;
const uint8_t ledChannel = 0;
const uint8_t resolution = 8;

/* Routine to swap double buffer pointers */
void IRAM_ATTR swap() {
  volatile uint16_t* temp_ptr = write_to;
  write_to = read_from;
  read_from = temp_ptr;
}

/* Timer interrupt routine */
void IRAM_ATTR onTimer() {
  static uint16_t idx = 0;
  BaseType_t task_woken = pdFALSE;

  if ((idx < BUF_LEN) && (buf_overrun == 0)) {              // If buffer isn't oberrun, accumulate buffer with ADC value
    write_to[idx] = adc1_get_raw(ADC1_CHANNEL_0);
    idx++;
  }

  if (idx >= BUF_LEN) {       
    if (xSemaphoreTakeFromISR(sem_done_reading, &task_woken) == pdFALSE) {      // If task has not taken semaphore and buffer is full, buffer has been overrun
      buf_overrun = 1;
    }

    if (buf_overrun == 0) {             // If sempaphore has been taken by task and value has been read, swap buffers and notify task
      idx = 0;
      swap();
      vTaskNotifyGiveFromISR(task_adc_avg, &task_woken);
    }
  }

    if (task_woken) {
      portYIELD_FROM_ISR();
    }
}

/* Task to average ADC values, compute PID algorithm and update PWM duty cycle */
void avgADC(void *params) {
    float avg;
    int x;
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);      // wait for notification from ISR
        avg = 0.0;
        for (int i = 0; i < BUF_LEN; i++) {
            avg += (float)read_from[i];
        }
        avg /= BUF_LEN;
        x = map(avg, 80, 640, 0, 255);                // map the adc value to fit DAC input size
        dac_output_voltage(DAC_ANGLE, x);
        portENTER_CRITICAL(&spinlock);                // protect adcVal variable from access during updation  
        adcVal = avg;
        buf_overrun = 0;
        myPID.compute();                              // compute PID algorithm
        xSemaphoreGive(sem_done_reading);             // give semaphore back after reading adcVal 
        portEXIT_CRITICAL(&spinlock);

        GPIO.out_w1ts = 1 << MOTOR_ENABLE;            // Motor enable pin is set HIGH
        ledcWrite(ledChannel, Output);                // PWM duty cycle is updated
    }
}

/* Task to listen to commands from Bluetooth serial */
void listen(void *params){
  char c;
  char buf[len];
  int8_t angle;                          
  uint8_t idx = 0;                       
  memset(buf, 0, len);               
    while (1) {
      if (SerialBT.available() > 0) {
        c = SerialBT.read();

      if (c == '\n') {                                  // if end of message
        buf[idx] = '\0'; 
        if (memcmp(buf, cmd, 4) == 0) {                 // check if entered message is "off"
          angle = -90;
          goto x;
        } else if (memcmp(buf, kp, 3) == 0) {           // check if entered message is "kp "
            portENTER_CRITICAL(&spinlock);
            Kp = atof(buf + 3);                         // update Kp with new value 
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } else if (memcmp(buf, kd, 3) == 0) {           // check if entered message is "kd "
            portENTER_CRITICAL(&spinlock);
            Kd = atof(buf + 3);                         // update Kd with new value
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } else if (memcmp(buf, ki, 3) == 0) {           // check if entered message is "ki "
            portENTER_CRITICAL(&spinlock);
            Ki = atof(buf + 3);                         // update Ki with new value
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } else if (memcmp(buf, b, 5) == 0) {            // check if entered message is "beta "
            portENTER_CRITICAL(&spinlock); 
            beta = atof(buf + 5);                       // update beta with new value
            myPID.SetGains(Kp, Ki, Kd, beta);
            portEXIT_CRITICAL(&spinlock);
            angle = angle_dup;
        } 
          else {
            angle = atoi(buf);
            if (angle > 50 || angle < -90){             // check if angle is out of range
		          SerialBT.println("Angle out of range");
              angle = angle_dup;
	         } else {
		          angle_dup = angle;
	          }
        }

        x : angle_dup = angle;
        portENTER_CRITICAL(&spinlock);
        Setpoint = 430 + angle*3.88;                   // compute setpoint in ADC units
        int y = map(Setpoint, 80, 640, 0, 255);
        portEXIT_CRITICAL(&spinlock);
        dac_output_voltage(DAC_SP, y);
        angleInput = angle; 
        memset(buf, 0, len);                           // clear buffer
        idx = 0;                   
      }
      
      if (idx < len - 1 && (isalnum(c) || c == '-' || c == ' ' || c == '.')) { 
        buf[idx] = c;
        idx++;
      }
    }
    /* Delay so it doesn't hog CPU time */
    vTaskDelay(2/portTICK_PERIOD_MS);
  }
}

void OLED(void *ignore) {
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda  = PIN_SDA;
  u8g2_esp32_hal.scl  = PIN_SCL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);              // configure I2C
  u8g2_t u8g2;
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  char kp[10], ki[10], kd[10], Beta[10];
  memset(kp, 0, 10); memset(ki, 0, 10); memset(kd, 0, 10); memset(Beta, 0, 10);

  while(1) {
    portENTER_CRITICAL(&spinlock);
    int16_t adc = adcVal;
    portEXIT_CRITICAL(&spinlock);

    sprintf(kp, "%0.3f", myPID.GetKp());                // convert float value into char[]
    sprintf(ki, "%0.3f", myPID.GetKi());
    sprintf(kd, "%0.3f", myPID.GetKd());
    sprintf(Beta, "%0.2f", myPID.GetBeta());
    
    adc = 0.257*(adc - 430);                            // map obtained ADC value into angle in degrees
    
    /* Align display text and symbols within 128x64 */
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_t0_11b_me);
    u8g2_DrawStr(&u8g2, 13, 17 , "Copter Parameters");
    u8g2_DrawHLine(&u8g2, 2, 20, 128);
    u8g2_DrawStr(&u8g2, 2, 32 ,"DC:");
    portENTER_CRITICAL(&spinlock);
    u8g2_DrawStr(&u8g2, 25, 32, u8x8_utoa(Output/255*100));         // u8x8 is used to convert uint to string
    portEXIT_CRITICAL(&spinlock);
    u8g2_DrawUTF8(&u8g2, 45, 32, "%");
    u8g2_DrawStr(&u8g2, 2, 43, "SP:");
    if (angleInput < 0) {                                           // if angle is < 0, add a "-" sign
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

    /* Send buffer to OLED display */
    u8g2_SendBuffer(&u8g2);

    /* Delay so it doesn't hog CPU time */
    vTaskDelay(200/portTICK_PERIOD_MS);
  } 
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("1 DoF copter");

  /* Configure ADC channel */
  adc1_config_width(ADC_WIDTH_10Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  /* Configure DAC channels */
  dac_output_enable(DAC_SP);
  dac_output_enable(DAC_ANGLE);

  /* Initialize PWM output channel */
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(PWM_PIN, ledChannel);

  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  /* Create a binary semaphore */
  sem_done_reading = xSemaphoreCreateBinary();

  /* Restart microcontoller if semaphore couldn't be created */
  if (sem_done_reading == NULL) {
        Serial.println("Could not create one or more semaphores");
        ESP.restart();
    }

  /* Give semaphore to initialize value to zero */
  xSemaphoreGive(sem_done_reading);

  /* Output saturation limits of PID should not exceed PWM duty cycle limits */
  myPID.limitOutput(0, 255);

  /* Create tasks pinned to specific cores*/
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
                          NULL,
                          0);

  xTaskCreatePinnedToCore(OLED,
                          "OLED Display",
                          2048,
                          NULL,
                          1,
                          NULL,
                          0);
  
  /* Initialize Timer 1 and attach an interrupt to it */
  timer = timerBegin(1, timer_divider, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timer_max_count, true);
  timerAlarmEnable(timer);
  vTaskDelete(NULL);    // Delete setup and loop tasks
}

void loop() {

}
