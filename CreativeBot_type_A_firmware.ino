/* 
 * ROS firmware for Me Auriga to controll CreativeBot type A
 * 
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int16.h>
#include "MeAuriga.h"

#define POWER_PORT                           A4
#define BUZZER_PORT                          45
#define RGBLED_PORT                          44


MeUltrasonicSensor ultraSensor(PORT_9);
MeSoundSensor mySound(A1);
MeLightSensor mylightSensor1(PORT_12);
MeLightSensor mylightSensor2(PORT_11);
MeOnBoardTemp OnBoardTemp(PORT_13);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeRGBLed led;
MeBuzzer buzzer;

ros::NodeHandle  nh;


sensor_msgs::Range range_msg;
std_msgs::Int16 sound_msg;
std_msgs::Int16 light1_msg;
std_msgs::Int16 light2_msg; 
sensor_msgs::Temperature temp_msg;
sensor_msgs::BatteryState battery_msg;
geometry_msgs::Twist command_velocity;
geometry_msgs::Vector3 command_buzzer;


ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::Publisher pub_sound( "/sound", &sound_msg);
ros::Publisher pub_light1( "/light1", &light1_msg);
ros::Publisher pub_light2( "/light2", &light2_msg);
ros::Publisher pub_temp( "/temp", &temp_msg);
ros::Publisher pub_battery( "/battery_state", &battery_msg);


long range_time;
long sound_time;
long light1_time;
long light2_time;
long temp_time;
long battery_time;
long led_time;
int r,g,b = 0;
bool motorOn = true;
float currentSpeed = 0;

const float pi = 3.141593;
const float two_pi = 6.283185;


/**********************************/
/* Callbacks                      */
void rgbCb( const std_msgs::ColorRGBA& rgb){
  // Blue
  led.setColor(rgb.r,rgb.g,rgb.b); 
  led.show();
}

void cmd_vel(const geometry_msgs::Twist& vel){

//  currentSpeed = mapfloat(vel.linear.x, -1.0, 1.0, -255.0, 255.0);
//
//  float speed_1_temp = currentSpeed;
//  float speed_2_temp = currentSpeed;
//
//  if(vel.linear.x > 0.0){
//    if(vel.angular.z >= 0.0){
//      speed_2_temp = speed_2_temp - (currentSpeed * vel.angular.z);
//    }
//    else if(vel.angular.z < 0.0){
//      speed_1_temp = speed_1_temp - (currentSpeed * -vel.angular.z);
//    }
//  }else if(vel.linear.x < 0.0){
//    if(vel.angular.z >= 0.0){
//      speed_1_temp = speed_1_temp + (-currentSpeed * vel.angular.z);
//    }
//    else if(vel.angular.z < 0.0){
//      speed_2_temp = speed_2_temp + (-currentSpeed * -vel.angular.z);
//    }
//  }

  
  // Distance between wheels are 145 mm
  // Wheel plus thicknes of track are 4 3mm in diameter 43 / 2 = 2.15 mm

  float angularCalc = vel.angular.z * 5.0;
  float linearCalc = vel.linear.x * 0.4;
  
  //float velocity_left_cmd = 1.0 * vel.linear.x + vel.angular.z * 0.145 / 2 ; 
  float rad_sec_left = (linearCalc - angularCalc * 0.145 / 2.0)/0.0215;
  
  //float velocity_right_cmd = 1.0 * vel.linear.x - vel.angular.z * 0.145 / 2; 
  float rad_sec_right = (linearCalc + angularCalc * 0.145 / 2.0)/0.0215; 

  float rpm_left = (60/two_pi) * rad_sec_left; 

  float rpm_right = (60/two_pi) * rad_sec_right;

  if(rpm_left > 180.0) {
    rpm_left = 180.0;
  }
  else if(rpm_left < -180.0){
    rpm_left = -180.0;
  }
  
  if(rpm_right > 180.0) {
    rpm_right = 180.0;
  }
  else if(rpm_right < -180.0){
    rpm_right = -180.0;
  }

//  if(velocity_left_cmd > 1.0) {
//    velocity_left_cmd = 1.0;
//  }
//  else if(velocity_left_cmd < -1.0){
//    velocity_left_cmd = -1.0;
//  }
//  
//  if(velocity_right_cmd > 1.0) {
//    velocity_right_cmd = 1.0;
//  }
//  else if(velocity_right_cmd < -1.0){
//    velocity_right_cmd = -1.0;
//  }
//
//
//  speed_1_temp = mapfloat(velocity_left_cmd, -1.0, 1.0, -255.0, 255.0);
//  speed_2_temp = mapfloat(velocity_right_cmd, -1.0, 1.0, -255.0, 255.0);
//  
  if(motorOn)
  {
    Encoder_1.runSpeed(-rpm_right);
    Encoder_2.runSpeed(rpm_left);
//    Encoder_1.runSpeed(-speed_1_temp);
//    Encoder_2.runSpeed(speed_2_temp);
    
//    Encoder_1.runSpeed();
//    Encoder_2.runSpeed(mapfloat(velocity_right_cmd, -1.0, 1.0, -255.0, 255.0));
  }
  else
  {
    Encoder_1.runSpeed(0);
    Encoder_2.runSpeed(0);
  }
}

void cmd_buzzer(const geometry_msgs::Vector3& val){
  if(val.x != 0){
    //buzzerOn();
    buzzer.tone(val.x, val.y);
  }else{
    buzzer.noTone(BUZZER_PORT);
    //buzzerOff();
  }
  //buzzer.tone(val.x, val.y);
}


void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}
/**********************************/


/**********************************/
/* Helper functions               */
/**********************************/
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void startup_seq()
{
  buzzer.tone(988, 200);

  for(int j=0; j<3; j++){
    for(int i=0; i<16; i++){
      led.setColor(0,0,0);
      led.setColor(i,0,0,255); 
      led.show();
      delay(10);
    }
  }
}

/**********************************/

ros::Subscriber<std_msgs::ColorRGBA> sub("rgb", &rgbCb );
ros::Subscriber<geometry_msgs::Twist> sub2("cmd_vel", &cmd_vel);
ros::Subscriber<geometry_msgs::Vector3> sub3("buzzer", &cmd_buzzer);

int16_t auriga_power = 0;

char frameid[] = "/ultrasound";
char frameidTemp[] = "/temp";
char frameidBattery[] = "/battery_state";


void setup()
{

  /* Sound sensor */
  mySound.setpin(A1);

  
  /* Buzzer */
  buzzer.setpin(BUZZER_PORT);


  /* LED Ring */
  led.setpin(RGBLED_PORT);
  led.setColor(0,0,0); 
  led.show();


  /* Ecoder motor drives */
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(0.18,0,0);
  Encoder_2.setPosPid(0.18,0,0);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
  Encoder_1.runSpeed(0);
  Encoder_2.runSpeed(0); 


 /* Show startup is on the way */
  startup_seq();

 
  nh.initNode();
  nh.advertise(pub_range);
  nh.advertise(pub_sound);
  nh.advertise(pub_light1);
  nh.advertise(pub_light2);
  nh.advertise(pub_temp);
  nh.advertise(pub_battery);

  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

  temp_msg.header.frame_id = frameidTemp;

  battery_msg.header.frame_id = frameidBattery;
  battery_msg.power_supply_health = 0;
  battery_msg.power_supply_status = 0;
  battery_msg.power_supply_technology = 1;
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 0.3;
  range_msg.max_range = 4.0;
  
  //pinMode(8,OUTPUT);
  //digitalWrite(8, LOW);
}



void loop()
{ 
  // publish the ultrasonic value every 50 milliseconds
  if ( millis() >= range_time ){
    range_msg.range = ultraSensor.distanceCm();
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_time =  millis() + 50;
  }

  // publish the battery value every 500 milliseconds
  if ( millis() >= battery_time ){
    battery_msg.voltage = get_power();
    battery_msg.header.stamp = nh.now();
    pub_battery.publish(&battery_msg);
    battery_time =  millis() + 50;
  }

  // publish the temp value every 500 milliseconds
  if ( millis() >= temp_time ){
    temp_msg.temperature = OnBoardTemp.readValue();
    temp_msg.header.stamp = nh.now();
    pub_temp.publish(&temp_msg);
    temp_time =  millis() + 50;
  }

  // publish the sound sensor strength every 10 milliseconds
  if(millis() >= sound_time){
    sound_msg.data = mySound.strength();
    pub_sound.publish(&sound_msg);
    sound_time = millis()+10;
  }

  // publish the light sensor 1 strength every 10 milliseconds
  if(millis() >= light1_time){
    light1_msg.data = mylightSensor1.read();
    pub_light1.publish(&light1_msg);
    light1_time = millis()+10;
  }

  // publish the light sensor 2 strength every 10 milliseconds
  if(millis() >= light2_time){
    light2_msg.data = mylightSensor2.read();
    pub_light2.publish(&light2_msg);
    light2_time = millis()+10;
  }

//  if(millis() >= led_time){
//    led.setColor(r,g,b); 
//    led.show();
//    led_time = millis()+10;
//  }

  if(!motorOn)
  {
    Encoder_1.runSpeed(0);
    Encoder_2.runSpeed(0);
  }

  Encoder_1.loop();
  Encoder_2.loop();
  nh.spinOnce();
}

/**
 * \par Function
 *    get_power
 * \par Description
 *    This function used to get the value of power supply
 * \param[in]
 *    None
 * \par Output
 *    None
 * \return
 *    The power vlaue(unit is V)
 * \par Others
 *    None
 */
float get_power(void)
{
  float power;
  auriga_power = analogRead(POWER_PORT);
  power = (auriga_power/1024.0) * 15;
  return power;
}
