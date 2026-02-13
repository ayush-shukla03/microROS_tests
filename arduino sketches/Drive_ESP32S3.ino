
/*
 * This is a simple template to use microros with ESP32-Arduino
 * This sketch has sample of publisher to publish from timer_callback
 * and subscrption to listen of new data from other ROS node.
 * 
 * Some of the codes below are gathered from github and forums
 * 
 * Made by Rasheed Kittinanthapanya
 * 
*/


/*
 * TODO : Include your necessary header here
*/


/////////////////////
/// For Micro ROS ///
/////////////////////
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <Cytron_SmartDriveDuo.h>
/*
 * TODO : include your desired msg header file
*/
#include <std_msgs/msg/float32_multi_array.h>

/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/
#define LED_PIN 23
#define IN1 19
#define IN2 20
#define IN3 21

Cytron_SmartDriveDuo motor_back(SERIAL_SIMPLIFIED, IN1, 115200);
Cytron_SmartDriveDuo motor_mid(SERIAL_SIMPLIFIED, IN2, 115200);
Cytron_SmartDriveDuo motor_front(SERIAL_SIMPLIFIED, IN3, 115200);

/*
 * Helper functions to help reconnect
*/
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

/*
 * Declare rcl object
*/
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/
rcl_subscription_t subscriber;

/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/

std_msgs__msg__Float32MultiArray msg;

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32MultiArray * msg0 = (const std_msgs__msg__Float32MultiArray *)msgin;
  msg.data.data[0]=msg0->data.data[0];
  msg.data.data[1]=msg0->data.data[1];
  msg.data.data[2]=msg0->data.data[2];
  msg.data.data[3]=msg0->data.data[3];
  msg.data.data[4]=msg0->data.data[4];
  msg.data.data[5]=msg0->data.data[5];
  //digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

/*
   Create object (Initialization)
*/
bool create_entities()
{
  /*
     TODO : Define your
     - ROS node name
     - namespace
     - ROS_DOMAIN_ID
  */
  const char * node_name = "esp32_boiler_plate_node";
  const char * ns = "";
  const int domain_id = 0;
  
  /*
   * Initialize node
   */
  allocator = rcl_get_default_allocator();
  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, domain_id);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, node_name, ns, &support);

  
  /*
   * TODO : Init your publisher and subscriber 
   */
  rclc_subscription_init(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "rover", &rmw_qos_profile_default);

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 1;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);


  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  rcl_subscription_fini(&subscriber, &node);
  
}

void setup() {
  /*
   * TODO : select either of USB or WiFi 
   * comment the one that not use
   */
  set_microros_transports();
  //set_microros_wifi_transports("WIFI-SSID", "WIFI-PW", "HOST_IP", 8888);

  /*
   * Optional, setup output pin for LEDs
   */
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);


  /*
   * TODO : Initialze the message data variable
   */

  //msg.data.data = (float_t*)malloc(6 * sizeof(float_t)); 
  float_t arr[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg.data.size = 6;
  msg.data.capacity = 6;
  msg.data.data = arr;

  /*
   * Setup first state
   */
  state = WAITING_AGENT;

}

void loop() {
  /*
   * Try ping the micro-ros-agent (HOST PC), then switch the state 
   * from the example
   * https://github.com/micro-ROS/micro_ros_arduino/blob/galactic/examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
   * 
   */
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
  /*
   * Output LED when in AGENT_CONNECTED state
   */
  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
    motor_back.control(msg.data.data[5], msg.data.data[4]);
    motor_mid.control(msg.data.data[3], msg.data.data[2]);
    motor_front.control(msg.data.data[1], msg.data.data[0]);
  } 
  else {
    digitalWrite(LED_PIN, 0);
    motor_back.control(0.0, 0.0);
    motor_mid.control(0.0, 0.0);
    motor_front.control(0.0, 0.0);
  }

  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

}