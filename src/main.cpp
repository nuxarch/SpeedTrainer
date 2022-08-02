#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SimpleFOC.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define TFT_CS  15
#define TFT_DC   2
#define TFT_MOSI  23
#define TFT_CLK   32
#define TFT_RST  4
#define TFT_MISO 5

TaskHandle_t taskDisplayHandle = NULL;
TaskHandle_t taskSensor1Handle = NULL;

long i = 0;

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST,TFT_MISO);
int sensor_tick = 100;

Encoder encoder1 = Encoder(27, 26, 30);
Encoder encoder2 = Encoder(35, 34, 30);
Encoder encoder3 = Encoder(25, 33, 30);

// interrupt routine intialisation
void doA1(){encoder1.handleA();}
void doB1(){encoder1.handleB();}
void doA2(){encoder2.handleA();}
void doB2(){encoder2.handleB();}
void doA3(){encoder3.handleA();}
void doB3(){encoder3.handleB();}

int dataSens1[350];
int dataSens2[350];
int dataSens3[350];


rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // msg.data++;
    
    msg.data = dataSens1[msg.data++];
    if (msg.data > 240) msg.data = 0;
  }
}
uint32_t Freq = 0;

void testFastLines(uint16_t color1, uint16_t color2) {
  // unsigned long sta3t;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  // start = micros();
  // for(y=25; y<=h;y+=25) tft.drawFastHLine(25, y, w-65, color1);
  
  for(x=0; x<=w; x+=50) {
    tft.drawFastVLine(x, 50, 5, color2);
    tft.setRotation(1);
    tft.setCursor(20, x+30);
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
    switch (x)
    {
    case 0:
      tft.println("12000");
      break;
    case 50:
      tft.println(" 9000");
      break;
    case 100:
      tft.println(" 6000");
      break;
    case 150:
      tft.println(" 3000");
      break;
    case 200:
      tft.println("    0");
      break;
    }
    tft.setRotation(0);
  }
  
  for(x=0; x<=w; x+=10) tft.drawFastVLine(x, 50, 2, color2);
  
  // return micros() - start;
}

void taskSensor1(void *parameter){
  for(;;){
    dataSens1[i] = encoder1.getAngle()+3;
    dataSens2[i] = encoder2.getAngle()+3;
    dataSens3[i] = encoder3.getAngle()+3;
    if (i >= 320){
      dataSens1[0] = dataSens1[320];
      dataSens2[0] = dataSens2[320];
      dataSens3[0] = dataSens3[320];
      i = 0;
    }
    if (i > 0){
      tft.drawFastHLine(0,i+59,240,ILI9341_BLACK);
      tft.drawLine(dataSens1[i-1], i-1+60,dataSens1[i], i+60,ILI9341_WHITE);
      tft.drawLine(dataSens2[i-1], i-1+60,dataSens2[i], i+60,ILI9341_RED);
      tft.drawLine(dataSens3[i-1], i-1+60,dataSens3[i], i+60,ILI9341_BLUE);
    }
    Serial.println("Speed : "+String(dataSens1[i]));
    vTaskDelay(10/portTICK_PERIOD_MS);
    ++i;
  }
}
void taskDisplay(void *parameter)
{
  tft.begin();
  testFastLines(ILI9341_CYAN, ILI9341_CYAN);
  xTaskCreate(taskSensor1, "task Sensor1", 10000, NULL, 0, &taskSensor1Handle);  

  while(1){
    vTaskDelay(500 / portTICK_PERIOD_MS);
    vTaskDelete(taskDisplayHandle);
  }
}
void setup() {
  Serial.begin(115200);
  encoder1.quadrature = Quadrature::OFF;
  encoder2.quadrature = Quadrature::OFF;
  encoder3.quadrature = Quadrature::OFF;
  encoder1.pullup = Pullup::USE_INTERN;
  encoder2.pullup = Pullup::USE_INTERN;
  encoder3.pullup = Pullup::USE_INTERN;
  encoder1.init();
  encoder2.init();
  encoder3.init();
  encoder1.enableInterrupts(doA1, doB1);  
  encoder2.enableInterrupts(doA2, doB2);  
  encoder3.enableInterrupts(doA3, doB3);
/*
  // set_microros_serial_transports(serial);
  IPAddress agent_ip(192,168,1,15);
  size_t agent_port = 8888;
  char ssid[] = "Ravensburg";
  char psk[]= "4khtar2015";
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
*/
  xTaskCreate(taskDisplay, "task Display", 4000, NULL, 1, &taskDisplayHandle);
}
void loop(void) {
  // delay(100);
  // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}
