#include <SPI.h>
#include <Controllino.h>
/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Bool& toggle_msg) {
  if (toggle_msg.data) {
    digitalWrite(CONTROLLINO_D0, HIGH);
  }
  else {
    digitalWrite(CONTROLLINO_D0, LOW);
  }
}

std_msgs::Bool vac_state;

ros::Subscriber<std_msgs::Bool> vacuum("toggle_vacuum", &messageCb );
ros::Publisher vac_switch("vacuum_state", &vac_state);

void setup()
{
  pinMode(CONTROLLINO_D0, OUTPUT);
//  pinMode(CONTROLLINO_IN0, INPUT_PULLUP);
//  attachInterrupt(CONTROLLINO_IN0, update, CHANGE);
  nh.initNode();
  nh.advertise(vac_switch);
  nh.subscribe(vacuum);
}

void loop()
{
  vac_state.data = digitalRead(CONTROLLINO_IN0);
  vac_switch.publish( &vac_state );
  nh.spinOnce();
  delay(100);
}

//void update() {
//  vac_state.data = digitalRead(CONTROLLINO_IN0);
//}
