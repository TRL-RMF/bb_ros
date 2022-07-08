#define SENSORPIN_L 7
#define SENSORPIN_R 12

#define USE_USBCON

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_srvs/SetBool.h>
using std_srvs::SetBool;

const uint16_t serverPort = 11411;
ros::NodeHandle nh;

std_msgs::Int16 arm_status_msg;
ros::Publisher arm_status("arm_status", &arm_status_msg);

std_msgs::Int16 trolley_dock_status_msg;
ros::Publisher trolley_dock_status("trolley_dock_status", &trolley_dock_status_msg);

void callback_srv(const SetBool::Request & req, SetBool::Response & res);

ros::ServiceServer<SetBool::Request, SetBool::Response> trolley_dock_server("trolley_lifting_arm_srv", &callback_srv);


const int IN1 = 2, IN2 = 3, PWM = 1;
const int sensorDown = 6, sensorUp = 4, SensorPinL = 7, sensorPinR = 12;

int user_input = 0, UP = 3, DOWN = 3; // a variable to read the encoder state

int sensorStateL = 2, sensorStateR = 2, sensorState = 2;

int position_state = 2, dock_state = 2;

int start_state = 0;

void messageDown()
{
  check_sensors();
  Serial.println("Step 4: Enterned MessageUp function, Value for DOWN is");
  Serial.println(DOWN);
  if (DOWN)
    Motor_Backward(200);
}


void setup()
{
  Serial.begin(9600);

  nh.initNode();
  nh.advertise(arm_status);
  nh.advertise(trolley_dock_status);
  nh.advertiseService(trolley_dock_server);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(sensorDown, INPUT);
  pinMode(sensorUp, INPUT);
  pinMode(SensorPinL, INPUT);
  pinMode(sensorPinR, INPUT);
  
  messageDown();
}

void loop()
{
  if ( start_state == 0)
  {
    while (!Serial);
    Serial.println("Step 1: Introduce, Enter your input for caato arm:");
    start_state = 1;
  }

  //arm_status_msg.data = 0; //1 is up, 0 is down
  arm_status_msg.data = position_state;
  arm_status.publish(&arm_status_msg);

  //trolley_dock_status_msg.data = 0; //1 docked, 0 is not docked
  trolley_dock_status_msg.data = dock_state;
  trolley_dock_status.publish(&trolley_dock_status_msg);
  nh.spinOnce();
  delay(100);
}

void messageUP()
{
  check_sensors();
  Serial.print("Step 4: Entered MessageUp function, Value for Up is");
  Serial.println(UP);
  check_sensors();
  if (UP)
    Motor_Forward(200);
}



void messageBR()
{
  check_sensors();
  Motor_Brake();
}

void Motor_Forward(int Speed) {
  while (1)
  {
    check_sensors();
    //Serial.println("Step 5: Motor forward function");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, Speed);
    if (UP == 0)
    {
      Motor_Brake();
      start_state = 0;
      delay(100);
      check_bb();
      if (sensorState == 0) {
        Serial.println("Docked");
        dock_state = 1;
        break;
      }
      else {
        Serial.println("Not Docked");
        messageDown();
        dock_state = 0;
        break;
      }
      break;
    }
  }
}

void Motor_Backward(int Speed) {
  while (1)
  {
    check_sensors();
    //    Serial.println("Motor backward function");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM, Speed);
    position_state = 0;
    if (DOWN == 0)
    {
      Motor_Brake();
      start_state = 0;
      dock_state = 0;
      break;
    }
  }

}

void Motor_Brake() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void check_sensors()
{
  UP = digitalRead(sensorUp);
  DOWN = digitalRead(sensorDown);
  if (UP && DOWN)
    position_state = 2;
  else if (UP && !DOWN)
    position_state = 0;
  else
    position_state = 1;
}

void check_bb()
{
  sensorStateL = digitalRead(SENSORPIN_L); //1 if not broken (not docked), 0 if broken (docked)
  sensorStateR = digitalRead(SENSORPIN_R); //1 if not broken (not docked), 0 if broken (docked)

  if (sensorStateL == 0 && sensorStateR == 0) { //docked
    sensorState = 0;
    dock_state = 1;
    //Serial.println("Docked");
  }
  else {
    sensorState = 1;
    dock_state = 0;
    //Serial.println("Not Docked");
  }
}

void callback_srv(const SetBool::Request & req, SetBool::Response & res) {

  if (position_state == req.data)
  {
    res.success = true;
    res.message = "already in position";
    return;
  }
  if (req.data)
  {
    messageUP();
    check_bb();
    if (dock_state == 1)
    {
      res.success = true;
      res.message = "dock successful";
      return;
    }
    else
    {
      res.success = false;
      res.message = "failed to dock";
      return;
    }

    res.success = false;
    res.message = "failed to dock";
    return;
    //if successful, res.success = true, res.message= ""
    //if failed, res.success = false, res.message= "type the issue here"
  }
  else
  {
    messageDown();
    res.success = true;
    res.message = "moved downwards";
  }
}
