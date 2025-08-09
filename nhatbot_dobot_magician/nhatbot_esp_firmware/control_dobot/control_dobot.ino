#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

bool agent_connected = false;

HardwareSerial mySerial(1); 

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 16, 17); 
  set_microros_transports();
  
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

    // Đợi agent tối đa 5 giây, mỗi 100ms
  for (int i = 0; i < 50; i++) {
    if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) { agent_connected = true; break; }
    delay(100);
  }
  if (!agent_connected) {
    // chớp nhanh báo chưa thấy agent nhưng vẫn thử init
    for (int i = 0; i < 10; i++) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 7)); // 👈 đổi số này nếu muốn

  // --- init support với init_options ---
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // --- giải phóng init_options ---
  RCCHECK(rcl_init_options_fini(&init_options));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),"micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default( &timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  // Nếu mất kết nối, thử ping và reset client để tự hồi
  if (rmw_uros_ping_agent(100, 1) != RCL_RET_OK) {
    // chớp LED chậm báo mất agent
    digitalWrite(LED_PIN, millis() / 250 % 2);
  } 
  else 
  {
    digitalWrite(LED_PIN, HIGH);
  }

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(50);
}