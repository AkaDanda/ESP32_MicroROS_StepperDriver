#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <WiFi.h>
#include <esp_now.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#include "Stepper.h" 

rcl_subscription_t subscriber;
std_msgs__msg__Int32 received_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

const int pin1N1 = 19;
const int pin1N2 = 18;
const int pin1N3 = 14;
const int pin1N4 = 15;  
#define STEPS 360

Stepper stepper(STEPS, pin1N1, pin1N2, pin1N3, pin1N4);

// MAC address of the receiver
uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x54, 0xAA, 0x6C};

// Structure example to send data
typedef struct struct_message {
    int data;
} struct_message;

struct_message myData;

// Callback function to handle incoming messages on the "keyboard_input" topic
void keyboard_input_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  int angle = msg->data;
  stepper.step(angle); // with angle =2048 it makes one rotation

  // Prepare data to send via ESP-NOW
  myData.data = 1;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Message sent successfully");
  } else {
    Serial.println("Error sending the message");
  }
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  stepper.setSpeed(60);
  
  // Set the signal pin as output
  pinMode(pin1N1, OUTPUT);
  pinMode(pin1N2, OUTPUT);
  pinMode(pin1N3, OUTPUT);
  pinMode(pin1N4, OUTPUT);

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "Keyboard_input"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &received_msg, &keyboard_input_callback, ON_NEW_DATA));

  // Initialize Wi-Fi in station mode
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  delay(1000);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
