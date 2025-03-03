/////////////////////////////////////////////////////////////////
/*
  Cracking password on Mac?? Brute-force USB Keyboard using ESP32-S2 (based on Keyset)
  Video Tutorial: https://youtu.be/AJ1lSk_aK6M
  Created by Eric N. (ThatProject)
*/
/////////////////////////////////////////////////////////////////


#include "Display.hpp"
#include <LovyanGFX.hpp>
#include <MCP3428.h>
#include <Wire.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#define LGFX_USE_V1

rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

MCP3428 MCP(0x68); // MCP3428 I2C address
static LGFX lcd;    // Display object

float val;

void error_loop(){
  while(1){

    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("ROS_DISCONNECT!", lcd.width() / 2, lcd.height() / 2, 4);
delay(100);
  }
}

// Function to map float values
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup()
{
    set_microros_transports();

    lcd.fillScreen(TFT_BLACK);
    lcd.drawString("Setup ...", lcd.width() / 2, lcd.height() / 2, 4);
    delay(2000);
    allocator = rcl_get_default_allocator();
    
    // Init MicroRos
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // Create node
    RCCHECK(rclc_node_init_default(&node, "Tension_Esp32_node", "", &support));

    // Create publisher
    RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "Tension_Esp32_node_publisher"));
    
    // Create timer
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
    msg.data = 0;
    lcd.init();
    lcd.setRotation(1);
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.setFont(&fonts::Font2);

}

void loop()
{
    byte error;
    int8_t address = MCP.devAddr;

    // Check if MCP3428 is detected
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
        long Raw_ADC;

        // Configure MCP3428 for Channel 1, 16-bit resolution, One-Shot mode, PGA=1
        MCP.SetConfiguration(1, 16, 0, 1);

        // Read ADC value
        Raw_ADC = MCP.readADC() * 0.0625; // 16-bit LSB = 62.5µV

        // Constrain & map force values
        val = constrain(Raw_ADC, 185, 950);
        val = mapfloat(val, 185, 950, 0, 20);


        // Update LCD Display
        lcd.fillScreen(TFT_BLACK); // Clear screen before updating
        lcd.setTextSize(2);
        lcd.setTextColor(TFT_YELLOW, TFT_BLACK);

        // Display Force Value (val) in Newtons
        lcd.drawString(String(val) + " N", lcd.width() / 2, lcd.height() / 3, 4);
        lcd.setTextColor(TFT_GREEN, TFT_BLACK);

        // Display ADC Value
        lcd.drawString("ADC: " + String(Raw_ADC), lcd.width() / 2, lcd.height() * 2 / 3, 4);
    }
    else
    {

        lcd.fillScreen(TFT_BLACK);
        lcd.drawString("MCP3428 Disconnected!", lcd.width() / 2, lcd.height() / 2, 4);
    }
    
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  
    delay(500); 
}
