// #include <font5x7.h>

//#include <micro_ros_arduino.h>

#include <stdio.h>
//#include <rcl/rcl.h>
//#include <rcl/error_handling.h>
//#include <rclc/rclc.h>
//#include <rclc/executor.h>

//#include <std_msgs/msg/u_int32.h>

// #include <ServoInvertible.h> 

// calls the function fn and runs the error loop if an error is detected
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

/***********************/
/*     Device Pins     */
/***********************/

#define THRUSTER_LEFT_FRONT  18
#define THRUSTER_LEFT_TOP  19
#define THRUSTER_LEFT_BACK  20
#define THRUSTER_RIGHT_FRONT  21
#define THRUSTER_RIGHT_TOP  22
#define THRUSTER_RIGHT_BACK  23

#define ERROR_LED_PIN 13

/***********************/
/*     MSG Decode      */
/***********************/

#define THRUSTER_DATA_BIT_SIZE 5

#define TLF_SHIFT_VAL 0*THRUSTER_DATA_BIT_SIZE
#define TLT_SHIFT_VAL 1*THRUSTER_DATA_BIT_SIZE
#define TLB_SHIFT_VAL 2*THRUSTER_DATA_BIT_SIZE
#define TRF_SHIFT_VAL 3*THRUSTER_DATA_BIT_SIZE
#define TRT_SHIFT_VAL 4*THRUSTER_DATA_BIT_SIZE
#define TRB_SHIFT_VAL 5*THRUSTER_DATA_BIT_SIZE

#define TLF_MASK 0x1f << TLF_SHIFT_VAL
#define TLT_MASK 0x1f << TLT_SHIFT_VAL
#define TLB_MASK 0x1f << TLB_SHIFT_VAL
#define TRF_MASK 0x1f << TRF_SHIFT_VAL
#define TRT_MASK 0x1f << TRT_SHIFT_VAL
#define TRB_MASK 0x1f << TRB_SHIFT_VAL
// #define THRUSTER_MASK 0x1f

#define ISOLATE_BITS(data, thruster) (data & thruster##_MASK) >> thruster##_SHIFT_VAL
// #define ISOLATE_BITS(data, thruster) (data >> thruster##_SHIFT_VAL) & THRUSTER_MASK
// uint32_t ISOLATE_BITS(uint32_t data, int thruster) 
// {
//   printf("truster shift val %d\n", thruster);
//   return (data >> thruster) & THRUSTER_MASK;
// }
// #define CONVERT_MICROSECONDS(bit_val) 1356 + bit_val*9
#define STATIONARY_US 1500
#define STATIONARY_LEVEL 16
#define F_MIN_LEVEL 17
#define B_MIN_LEVEL 15
// Assume voltage draw 10V, max current 4A
#define F_MAX_US 1712 // 1736 // Max Forward thrust: 1.31 kgf
#define B_MAX_US 1276 // 1264 // Max Backward thrust: 1.06 kgf
#define F_MIN_US 1544 // Min Forward thrust: 0.04 kgf
#define B_MIN_US 1456 // Min Backward thrust: 0.03 kgf
#define F_FACTOR 12 // (F_MAX - F_MIN) / 14 = 13.71... ~= 12 (a multiple of 4) so F_MAX is actually F_MIN + F_FACTOR * 14 = 1712
#define B_FACTOR 12 // (B_MIN - B_MAX) / 15 = 12.8... ~= 12 (a multiple of 4) so B_MAX is actually B_MIN - B_FACTOR * 15 = 1276

#define CONVERT_MICROSECONDS(bit_val) \
(bit_val == STATIONARY_LEVEL ? \
 STATIONARY_US : \
 (bit_val > STATIONARY_LEVEL ? \
 (F_MIN_US + F_FACTOR * (bit_val - F_MIN_LEVEL)) : \
 (B_MIN_US - B_FACTOR * (B_MIN_LEVEL - bit_val))))

/***********************/
/*    Calib Consts     */
/***********************/

#define PWM_STOP        91
#define PWM_STATIC_MOVE 50

/***********************/
/*   Servo Objects     */
/***********************/

ServoInvertible TLF; //Thruster on left angled at front
ServoInvertible TLT; //Thruster on left pointing upwards
ServoInvertible TLB; //Thruster on left angled at back
ServoInvertible TRF; //Thruster on right angled at front
ServoInvertible TRT; //Thruster on right pointing upwards
ServoInvertible TRB; //Thruster on right angled at back 

/***********************/
/*        E-Stop       */
/***********************/
#define E_STOP_ANALOG_PIN 0

// Digital threshold value for voltage under which
// we will consider the e-stop to be triggered.
// Value is from: 
// (ADC_VALUE = 1023 / 5 * THRESHOLD_VOLTAGE)
#define E_STOP_ADC_THRESHOLD 204 

bool e_stop_triggered = false;

/***********************/
/*         ROS         */
/***********************/
//rcl_subscription_t subscriber;
//rcl_publisher_t debug_publisher;
//std_msgs__msg__UInt32 msg;
//std_msgs__msg__UInt32 out_msg;
//rclc_executor_t executor;
//rclc_support_t support;
//rcl_allocator_t allocator;
//rcl_node_t node;
//rcl_timer_t timer;

// Structure to store thruster PWM data
struct thruster_data {
  uint32_t tlf;
  uint32_t tlt;
  uint32_t tlb;
  uint32_t trf;
  uint32_t trt;
  uint32_t trb;
};

uint32_t read_buffer;

struct thruster_data pwm_data;

/* 
 * Decodes from message bitwise representation to a pulse length in 
 * microseconds for each thruster.
 *
 * Data is packed within the 32 bit integers in groups of 5 bits (i.e. TLF 
 * data occupies bits 0-4, TLT data occupies bits 5-9, etc.). 
 *
 * The corrects bits are isolated for each thurster with a bit mask and bit 
 * shift, and then mapped linearly to the range 1100-1900 microseconds.
 */
void decode_thruster_data(uint32_t msg_data) {
  uint32_t tlf_bits = ISOLATE_BITS(msg_data, TLF);
  uint32_t tlt_bits = ISOLATE_BITS(msg_data, TLT);
  uint32_t tlb_bits = ISOLATE_BITS(msg_data, TLB);
  uint32_t trf_bits = ISOLATE_BITS(msg_data, TRF);
  uint32_t trt_bits = ISOLATE_BITS(msg_data, TRT);
  uint32_t trb_bits = ISOLATE_BITS(msg_data, TRB);

  pwm_data.tlf = CONVERT_MICROSECONDS(tlf_bits);
  pwm_data.tlt = CONVERT_MICROSECONDS(tlt_bits);
  pwm_data.tlb = CONVERT_MICROSECONDS(tlb_bits);
  pwm_data.trf = CONVERT_MICROSECONDS(trf_bits);
  pwm_data.trt = CONVERT_MICROSECONDS(trt_bits);
  pwm_data.trb = CONVERT_MICROSECONDS(trb_bits);
}

/*
 * Flash LED on board in case of error
 */
void error_loop(){
  while(1){
    digitalWrite(ERROR_LED_PIN, !digitalRead(ERROR_LED_PIN));
    delay(100);
  }
}

/**
 * Receives UInt32 messages from the Controller Node
 * The robot has five degrees of freedom: 
 * Linear x, y, z, and yaw
 * 
 * The robot can move in different combinations of 
 * movement. The straight motors allow it to exclusively 
 * move in either yaw or linear x. The angled motors 
 * allow it to exclusively move in either linear y or z.
 */
void motorControlCallback (uint32_t msg_data){
  
  // convert from bit representation to PWM pulse width in microseconds
  decode_thruster_data(msg_data);

  //   send data to thursters
  TLF.writeMicroseconds(pwm_data.tlf, 1);
  TLT.writeMicroseconds(pwm_data.tlt, 1);
  TLB.writeMicroseconds(pwm_data.tlb, 1);
  TRF.writeMicroseconds(pwm_data.trf, 1);
  TRT.writeMicroseconds(pwm_data.trt, 1);
  TRB.writeMicroseconds(pwm_data.trb, 1);

  // set published message to be the PWM signal sent to TLF
  //out_msg.data = pwm_data.tlf;

  // publish data via debug_publisher
  //RCSOFTCHECK(rcl_publish(&debug_publisher, &out_msg, NULL));
}

/***********************/
/*       Teensy        */
/***********************/

void setup()  {
  
  //set_microros_transports();

  /*
   * this will initialize PWM signal control on the specified pins. Default 
   * signal is 1500 microseconds.
   */
  TLF.attach(THRUSTER_LEFT_FRONT);
  TLT.attach(THRUSTER_LEFT_TOP);
  TLB.attach(THRUSTER_LEFT_BACK);
  TRF.attach(THRUSTER_RIGHT_FRONT);
  TRT.attach(THRUSTER_RIGHT_TOP);
  TRB.attach(THRUSTER_RIGHT_BACK);

  delay(2000);
  
  /*
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor_control_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "motor_control"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &debug_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "debug_publisher"));
 
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &motorControlCallback, ON_NEW_DATA));
  */
  
  //   init sequence
  TLF.writeMicroseconds(1500, 1);
  TLT.writeMicroseconds(1500, 1);
  TLB.writeMicroseconds(1500, 1);
  TRF.writeMicroseconds(1500, 1);
  TRT.writeMicroseconds(1500, 1);
  TRB.writeMicroseconds(1500, 1);

  //out_msg.data = TLF.readMicroseconds();
//   out_msg.data = msg->data;

  delay(7000);
  // publish data via debug_publisher
  //RCSOFTCHECK(rcl_publish(&debug_publisher, &out_msg, NULL));
}

void loop(SerialClass Serial)  {
  // Don't do anything if e-stop triggered
  if (!e_stop_triggered) {
    // Check if e-stop triggered
    int current_e_stop_value = analogRead(E_STOP_ANALOG_PIN);
    if (current_e_stop_value < E_STOP_ADC_THRESHOLD){
      e_stop_triggered = true;
    }
    else {
      e_stop_triggered = false;
    }
  }

  delay(100);

  while (Serial.available()>=4){
    Serial.readBytes((char *)&read_buffer, 4);
  }
  motorControlCallback(read_buffer);

}
