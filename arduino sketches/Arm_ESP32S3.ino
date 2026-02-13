
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
/*
 * TODO : include your desired msg header file
*/
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
/*
 * Optional,
 * LED pin to check connection
 * between micro-ros-agent and ESP32
*/
#define LED_PIN 23
#define DEADZONE 10
int l1_dir=1;
int l1_speed=2;
int l2_dir=4;
int l2_speed=5;
int base_dir=6;
int base_speed=7;
int finger_dir=8;
int finger_speed=9;
int bevel1_dir=10;
int bevel1_speed=11;
int bevel2_dir=12;
int bevel2_speed=13;
#define DROP_TOP_PIN 14
#define DROP_BOTTOM_PIN 17
int dropmotor_dir=18;
int dropmotor_speed=21;

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

int l1 = 0;
int l2 = 0;
int base = 0;
int gripper = 0;
int rbevel = 0;
int lbevel = 0;

/*
 * Declare rcl object
*/
rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;

/*
 * TODO : Declare your 
 * publisher & subscription objects below
*/

rcl_publisher_t publisher;
rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
/*
 * TODO : Define your necessary Msg
 * that you want to work with below.
*/
std_msgs__msg__Int32 drop_status;
std_msgs__msg__Int32MultiArray msg;
std_msgs__msg__Int32 msg_drop;

/*
 * TODO : Define your subscription callbacks here
 * leave the last one as timer_callback()
*/
void subscription_callback1(const void * msgin)
{  
  const std_msgs__msg__Int32MultiArray * msg0 = (const std_msgs__msg__Int32MultiArray*)msgin;
  l1=msg0->data.data[0];
  l2=msg0->data.data[1];
  lbevel=msg0->data.data[2];
  rbevel=msg0->data.data[3];
  base=msg0->data.data[4];
  gripper=msg0->data.data[5];
  //digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  
}
void subscription_callback2(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg0 = (const std_msgs__msg__Int32*)msgin;
  msg_drop.data=msg0->data;


  //digitalWrite(LED_PIN, (msg->data.data[0] == 0) ? LOW : HIGH);  
}

void timer_callback(rcl_timer_t* timer,int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {

    /*
       TODO : Publish anything inside here
       
       For example, we are going to echo back
       the int16array_sub data to int16array_pub data,
       so we could see the data reflect each other.
       And also keep incrementing the int16_pub
    */
    rcl_publish(&publisher, &drop_status, NULL);

  }
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
  rclc_publisher_init(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "drop_status", &rmw_qos_profile_default);


  rclc_subscription_init(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "mymotors", &rmw_qos_profile_default);

  rclc_subscription_init(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "dropmotor", &rmw_qos_profile_default);

  /*
   * Init timer_callback
   * TODO : change timer_timeout
   * 50ms : 20Hz
   * 20ms : 50Hz
   * 10ms : 100Hz
   */
  const unsigned int timer_timeout = 1500;
  rclc_timer_init_default(&timer,&support, RCL_MS_TO_NS(timer_timeout), timer_callback);

  /*
   * Init Executor
   * TODO : make sure the num_handles is correct
   * num_handles = total_of_subscriber + timer
   * publisher is not counted
   * 
   * TODO : make sure the name of sub msg and callback are correct
   */
  unsigned int num_handles = 3;
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, num_handles, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber1, &msg, &subscription_callback1, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subscriber2, &msg_drop, &subscription_callback2, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}
/*
 * Clean up all the created objects
 */
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_init_options_fini(&init_options);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  /*
   * TODO : Make sue the name of publisher and subscriber are correct
   */
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&subscriber1, &node);
  rcl_subscription_fini(&subscriber2, &node);
  
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
  pinMode(l1_dir, OUTPUT);
  pinMode(l1_speed, OUTPUT);
  pinMode(l2_dir, OUTPUT);
  pinMode(l2_speed, OUTPUT);
  pinMode(base_dir, OUTPUT);
  pinMode(base_speed, OUTPUT);
  pinMode(finger_dir, OUTPUT);
  pinMode(finger_speed, OUTPUT);
  pinMode(bevel1_dir, OUTPUT);
  pinMode(bevel1_speed, OUTPUT);
  pinMode(bevel2_dir, OUTPUT);
  pinMode(bevel2_speed, OUTPUT);
  pinMode(dropmotor_dir,OUTPUT);
  pinMode(dropmotor_speed,OUTPUT);
  pinMode(DROP_TOP_PIN, INPUT_PULLUP);
  pinMode(DROP_BOTTOM_PIN, INPUT_PULLUP);
  /*
   * TODO : Initialze the message data variable
   */
  int32_t array_data[6] = {0, 0, 0, 0, 0, 0};
  msg.data.capacity = 6; // number of array length
  msg.data.size = 6;     // number of array length
  msg.data.data = array_data;

  msg_drop.data=2; 

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

      //link1
      if(msg_drop.data==1)
    {
      if(digitalRead(DROP_TOP_PIN)==LOW)
      {
        digitalWrite(dropmotor_dir,HIGH);
        analogWrite(dropmotor_speed,0);
        drop_status.data=1;
      }
      else{
        digitalWrite(dropmotor_dir,HIGH);
        analogWrite(dropmotor_speed,255);
        drop_status.data=2;
      }
    }
    else if (msg_drop.data==0)
    {
      if(digitalRead(DROP_BOTTOM_PIN)==LOW)
      {
        digitalWrite(dropmotor_dir,LOW);
        analogWrite(dropmotor_speed,0);
        drop_status.data=0;
      }
      else{
        digitalWrite(dropmotor_dir,LOW);
        analogWrite(dropmotor_speed,255);
        drop_status.data=2;
      }
    }
    else
    {
      digitalWrite(dropmotor_dir,HIGH);
      analogWrite(dropmotor_speed,0);
    }
    if(l1>0)
    {
      digitalWrite(l1_dir,HIGH);
      analogWrite(l1_speed,l1);
    }
    else
    {
      digitalWrite(l1_dir,LOW);
      analogWrite(l1_speed,-l1);
    }
    
    //link2
    if(l2>0)
    {
      digitalWrite(l2_dir,LOW);
      analogWrite(l2_speed,l2);
    }
    else
    {
      digitalWrite(l2_dir,HIGH);
      analogWrite(l2_speed,-l2);
    }
  
    //base_yaw
    if(base>0)
    {
      digitalWrite(base_dir,HIGH);
      analogWrite(base_speed,base);
    }
    else if(base<0)
    {
      digitalWrite(base_dir,LOW);
      analogWrite(base_speed,-base);
    }
    else
    {
      digitalWrite(base_dir,LOW);
      analogWrite(base_speed,0);
    }

    //end_effector
    if(gripper>0) {
      digitalWrite(finger_dir,HIGH);
      analogWrite(finger_speed,gripper);
    }
    else if(gripper<0)
    {
      digitalWrite(finger_dir,LOW);
      analogWrite(finger_speed,-gripper);
    }
    else
    {
      digitalWrite(finger_dir,LOW);
      analogWrite(finger_speed,0);
    }

    if(abs(lbevel) < DEADZONE){
      analogWrite(bevel1_speed, 0);
      analogWrite(bevel2_speed, 0);
    }

    else{
      //bevel left
    if(lbevel>0)
    {
      digitalWrite(bevel1_dir, LOW);
      analogWrite(bevel1_speed,lbevel);
    }
    else
    {
      digitalWrite(bevel1_dir,HIGH);
      analogWrite(bevel1_speed,-lbevel);
    }
      
      //bevel right 
    if(rbevel>0)
    {
      digitalWrite(bevel2_dir,LOW);
      analogWrite(bevel2_speed,rbevel);
    }
    else
    {
      digitalWrite(bevel2_dir,HIGH);
      analogWrite(bevel2_speed,-rbevel);
    }

    }
  } 
  else {
    digitalWrite(LED_PIN, 0);
  }

  /*
   * TODO : 
   * Do anything else you want to do here,
   * like read sensor data,  
   * calculate something, etc.
   */

}