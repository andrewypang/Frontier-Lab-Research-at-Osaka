#include <DynamixelSDK.h>
#include <IMU.h>
/*
 * Info for Gyro
 * 
 *--Angular Acceleration
 *  Range   : +/- 2000 deg/sec
 *  Scale   : 16.4 = 1 deg/sec
 *
 *--Roll Pitch Yaw
 *  Range   : Roll  : +/- 180 deg/sec
 *            Pitch : +/- 180 deg/sec
 *            Yaw   : +/- 180 deg/sec
 *  Scale   : Roll  : 1 = 1 deg/sec
 *            Pitch : 1 = 1 deg/sec
 *            Yaw   : 1 = 1 deg/sec
 *  IMU.rpy[0] -> Roll
 *  IMU.rpy[1] -> Pitch
 *  IMU.rpy[2] -> Yaw
 */

// Control table address
#define ADDR_PRO_OPER_MODE              11
#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_PRESENT_POSITION       132
#define ADDR_PRO_GOAL_POS               116
#define ADDR_PRO_MAX_POS_LIMIT          48
#define ADDR_PRO_MIN_POS_LIMIT          52

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// OpenCR Settings & Default Settings
#define BAUDRATE                        3000000
#define DEVICENAME                      "3"                 // Check which port is being used on your controller
                                                            // ex) Serial1: "1"   Serial2: "2"   Serial3: "3
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define ESC_ASCII_VALUE                 0x1b

#define BDPIN_LED_USER_1        22
#define BDPIN_LED_USER_2        23
#define BDPIN_LED_USER_3        24
#define BDPIN_LED_USER_4        25
#define BDPIN_PUSH_SW_1         34
#define BDPIN_PUSH_SW_2         35

#define LINKCOUNT               8

cIMU    IMU;
bool gyroInitialize = false;

int led_pin_user[4] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4 };
int push_button_1_toggle = 0;                               // counter for the number of button presses
int push_button_1_current_state = 0;                        // current state of the button
int push_button_1_last_state = 0;                           // previous state of the button

bool gripper_toggle = false; //gripper closed => true; gripper open => false;
int gripper_face_state_toggle = 1; // 0 => off; 1 => face down; 2 => face up-right
bool gyro_toggle = false;

uint8_t dxl_error = 0;                          // Dynamixel error

//NOTE: jointAngles entry ID =/= Dynamixel Motor ID
uint32_t jointAngles[8]     = {2045,2045,2045,2045,2045,1704,2045,2045}; 
uint32_t jointAngles_Max[8] = {3863,3067,3800,2045,3800,3070,3863,3050};
uint32_t jointAngles_Min[8] = {250,1022,240 ,240 ,250 ,1027,227 ,2050};

uint32_t object_size = 4;

// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler;

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler;

float PosToDeg(float pos)
{
  return pos*0.088;
}
float DegToPos(float deg)
{
  return deg/0.088;
}

void initializeArm()
{
  // Set up Max Position Limit
  for(uint32_t id = 0, servo_id = 2; id < 8; id++, servo_id++)
  {
    packetHandler->write4ByteTxRx(portHandler, servo_id, ADDR_PRO_MAX_POS_LIMIT, jointAngles_Max[id], &dxl_error);
  }

  // Set up Min Position Limit
  for(uint32_t id = 0, servo_id = 2; id < 8; id++, servo_id++)
  {
    packetHandler->write4ByteTxRx(portHandler, servo_id, ADDR_PRO_MIN_POS_LIMIT, jointAngles_Min[id], &dxl_error);
  }

  // Read Current Joint Pos and load into jointAngles
  for(uint32_t id = 0, servo_id = 2; id < 8; id++, servo_id++)
  {
    int32_t dxl_present_position = 0;               // Present position
    packetHandler->read4ByteTxRx(portHandler, servo_id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    jointAngles[id] = dxl_present_position;
  }

  // Set all Dynamixel Motors to Operating Mode: Position Control Mode
  for(int id = 2; id <= 9; id++)
  {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_OPER_MODE, 3, &dxl_error);
  }
}

void resetInitialGyroArm()
{
  gyroInitialize = true;
  gripper_toggle = true; //gripper closed => true; gripper open => false;
}

bool isAngleInRange(uint32_t *src)
{
  for(uint32_t i = 0; i < 8; i++)
  {
    if(src[i] < jointAngles_Min[i] || src[i] > jointAngles_Max[i])
    {
      Serial.print("Joint ID not in range: ");Serial.println(i);
      return false;
    }
  }
  return true;
}

void moveServo(uint32_t servo_ID, uint32_t goalPos)
{
  uint32_t joint_ID = servo_ID - 2;

  // Check max & min then move servo
  if(goalPos >= jointAngles_Min[joint_ID] && goalPos <= jointAngles_Max[joint_ID])
  {
    packetHandler->write4ByteTxRx(portHandler, servo_ID, ADDR_PRO_GOAL_POS, goalPos, &dxl_error);
    delay(10);
  }
  else
  {
    Serial.println("LIMIT REACHED");
    Serial.print("Joint ID:"); Serial.print(joint_ID);
  }
  if (dxl_error != 0)
  {
    Serial.print(packetHandler->getRxPacketError(dxl_error));
  }
}

void moveAllPositions(uint32_t *posArrayToMoveTo)
{
  for(int id = 0, servo_id = 2; id < 8; id++, servo_id++)
  {
    moveServo(servo_id, (uint32_t)posArrayToMoveTo[id]);
  }
}

void updateAllPresentPositions(uint32_t *posArrayToUpdateFrom)
{
  for(int i = 0; i < LINKCOUNT; i++)
  {
    jointAngles[i] = posArrayToUpdateFrom[i];
  }
}

void setInitialGyroArmPos()
{
  if(gyroInitialize == true)
  {
    moveServo(8, (uint32_t)2045);
    delay(500);
    moveServo(6, (uint32_t)2045);
    delay(500);
    moveServo(4, (uint32_t)2045);
    delay(500);
    moveServo(9, (uint32_t)jointAngles_Max[7]);
    delay(500);
    gyroInitialize = false;
  }
}

void gyroControl()
{
  digitalWrite(led_pin_user[3], HIGH);

  setInitialGyroArmPos();

  float gyroAboutX_Axis = IMU.rpy[0]; //Rotate about the X axis for vertical  up-down movement - config to ID 5
  float gyroAboutY_Axis = IMU.rpy[1]; //Rotate about the Y axis for horizonal left-right movement - config to ID 2
  float gyroAboutZ_Axis = IMU.rpy[2]; //Rotate about the Z axis for wrist-twist movement - config to ID 8

  float delta_x = 0;
  float delta_y = 0;
  float delta_z = 0;

  uint32_t current_pos_ID2 = 0;
  uint32_t current_pos_ID5 = 0;
  uint32_t current_pos_ID8 = 0;

  packetHandler->read4ByteTxRx(portHandler, 2, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&current_pos_ID2, &dxl_error);
  packetHandler->read4ByteTxRx(portHandler, 5, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&current_pos_ID5, &dxl_error);
  packetHandler->read4ByteTxRx(portHandler, 8, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&current_pos_ID8, &dxl_error);

  float damping = 0.1;

  if((gyroAboutY_Axis < 90 || gyroAboutY_Axis > -90) && (gyroAboutY_Axis > 20 || gyroAboutY_Axis < -20)) //Only true if within range
  {
    delta_x = gyroAboutY_Axis;
    delta_x = DegToPos(delta_x) * damping;
    current_pos_ID2 = current_pos_ID2 + delta_x;
    
    //Use data to move arm
    moveServo(2, current_pos_ID2);
  }

  if((gyroAboutX_Axis < 90 || gyroAboutX_Axis > -90) && (gyroAboutX_Axis > 25 || gyroAboutX_Axis < -25)) //Only true if within range
  {
    delta_y = gyroAboutX_Axis;
    delta_y = (float)(DegToPos(delta_y) * -1.0*damping);
    current_pos_ID5 = current_pos_ID5 + delta_y;
    
    //Use data to move arm
    moveServo(5, current_pos_ID5);

    // Gravity compensation to deduce bounce feedback
    if(delta_y < 0)
    {
      float gravity_comp = abs(delta_y);
      //gravity_comp = gravity_comp/2.0;
      moveServo(5, current_pos_ID5+gravity_comp);
      delay(50);
    }
  }

  if((gyroAboutZ_Axis < 90 || gyroAboutZ_Axis > -90) && (gyroAboutZ_Axis > 30 || gyroAboutZ_Axis < -30)) //Only true if within range
  {
    delta_z = gyroAboutZ_Axis;
    delta_z = DegToPos(delta_z) * damping;
    current_pos_ID8 = current_pos_ID8 + delta_z;
    
    //Use data to move arm
     moveServo(8, current_pos_ID8);
  }

  switch(gripper_face_state_toggle)
  {
    case 1: fixGripperFaceDown();
            break;

    case 2: fixGripperFaceUpRight();
            break;

    case 0:
    default:;
  }

  //Push button 2 when LED 1 is on to open/close gripper
  if(digitalRead(BDPIN_PUSH_SW_2) == HIGH)
  {
    gripper_toggle = !gripper_toggle;
    operateGripper();
  }

  //For Debugging purposes
  bool debugGyro = false;
  if(debugGyro)
  {
    Serial.print("\tDelta_x-IN DEG:");Serial.print(gyroAboutY_Axis);Serial.print("\n");
    Serial.print("\tDelta_x-IN POS:");Serial.print(delta_x);Serial.print("\n");

    Serial.print("\tDelta_y-IN DEG:");Serial.print(gyroAboutX_Axis);Serial.print("\n");
    Serial.print("\tDelta_y-IN POS:");Serial.print(delta_y);Serial.print("\n");

    Serial.print("\tDelta_z-IN DEG:");Serial.print(gyroAboutZ_Axis);Serial.print("\n");
    Serial.print("\tDelta_z-IN POS:");Serial.print(delta_z);Serial.print("\n");
  }

  digitalWrite(led_pin_user[3], LOW); 
}

//Toggle Gripper to face down position
void fixGripperFaceDown()
{
  uint32_t presentPositionID_5;
  uint32_t presentPositionID_3;
  
  packetHandler->read4ByteTxRx(portHandler, 5, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&presentPositionID_5, &dxl_error);
  packetHandler->read4ByteTxRx(portHandler, 3, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&presentPositionID_3, &dxl_error);

  presentPositionID_3 = presentPositionID_3 - DegToPos(90); //Get relative Position
  
  uint32_t total_pos = DegToPos(270); //For Two Joint Movement - Servo_ID5 & Servo_ID3
  
  uint32_t ID7_offset = DegToPos(0);
  
  uint32_t newPosForID7 = total_pos - presentPositionID_3 - presentPositionID_5;

  moveServo(7, newPosForID7);
  
}

//Toggle Gripper to up-right position
//Same code as fixGripperFaceDown() but add 90 degrees at the end
void fixGripperFaceUpRight()
{
  uint32_t presentPositionID_5;
  uint32_t presentPositionID_3;
  
  packetHandler->read4ByteTxRx(portHandler, 5, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&presentPositionID_5, &dxl_error);
  packetHandler->read4ByteTxRx(portHandler, 3, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&presentPositionID_3, &dxl_error);

  presentPositionID_3 = presentPositionID_3 - DegToPos(90); //Get relative Position
  
  uint32_t total_pos = DegToPos(270); //For Two Joint Movement - Servo_ID5 & Servo_ID3
  
  uint32_t ID7_offset = DegToPos(0);
  
  uint32_t newPosForID7 = total_pos - presentPositionID_3 - presentPositionID_5;

  moveServo(7, newPosForID7 + DegToPos(90));
}

void operateGripper()
{
  if(gripper_toggle)
  {
    //Open Gripper
    moveServo(9, jointAngles_Max[7]);
  }
  else
  {
    //Close Gripper
    if(object_size > 0)
    {
      uint32_t clawCloseObjectWidth = object_size * 50;
      moveServo(9, jointAngles_Min[7]+clawCloseObjectWidth);
    }
    else
    {
      //Default Claw Close Width
      moveServo(9, jointAngles_Min[7]);
    }
    
  }    
  delay(100);
}

void setup(){

  //Setup pins for button switches and LEDs
  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);
  
  pinMode(led_pin_user[0], OUTPUT);
  pinMode(led_pin_user[1], OUTPUT);
  pinMode(led_pin_user[2], OUTPUT);
  pinMode(led_pin_user[3], OUTPUT);

  digitalWrite(led_pin_user[0], LOW);

  //Initialize the handlers
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  //Load JointAngles Max/Min and more into Dynamixel Motors
  initializeArm();
  
  Serial.begin(115200);

  Serial.println("Start..");

  // Open port
  if (portHandler->openPort())
  {
    Serial.println("Succeeded to open the port!");
  }
  else
  {
    Serial.println("Failed to open the port!");
    return;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    Serial.print("Succeeded to change the baudrate to ");
    Serial.print(portHandler->getBaudRate());
    Serial.print("\n");
  }
  else
  {
    Serial.print("Failed to change the baudrate!\n");
    return;
  }

  digitalWrite(led_pin_user[1], LOW);
    
  IMU.begin();
  Serial.println("ACC/Gyro Cali Start");
  IMU.SEN.acc_cali_start();
  IMU.SEN.gyro_cali_start();
  
  digitalWrite(led_pin_user[2], LOW);
  
  while( IMU.SEN.gyro_cali_get_done() == false  || IMU.SEN.acc_cali_get_done() == false)
  {
    IMU.update();
  }
  Serial.print("ACC/Gyro Cali End ");

  digitalWrite(led_pin_user[3], LOW);

  //Flash LEDs to indicate OpenCR setup() is finished
  for(int i=0; i<4; i++ )
  {
    digitalWrite(led_pin_user[i], HIGH);
    delay(100);
  }
  for(int i=3; i>=0; i--)
  {
    digitalWrite(led_pin_user[i], LOW);
    delay(100);
  }
  for(int i=3; i>=0; i--)
  {
    digitalWrite(led_pin_user[i], HIGH);
    delay(100);
  }
   
}
void loop(){

  //Use Serial Monitor to toggle options
  if( Serial.available() )
  {
    char Ch = Serial.read();

    if(Ch == '0')
    {
      Serial.println("ACC/Gyro Cali Start");
      IMU.SEN.acc_cali_start();
      IMU.SEN.gyro_cali_start();
      while( IMU.SEN.gyro_cali_get_done() == false  || IMU.SEN.acc_cali_get_done() == false)
      {
        IMU.update();
      }
      Serial.print("ACC/Gyro Cali End ");
      
    }
    else if(Ch == '1')
    {
      gripper_face_state_toggle = 1;
    }
    else if(Ch == '2')
    {
      gripper_face_state_toggle = 2;
    }
  }

  IMU.update();
  
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  push_button_1_current_state = digitalRead(BDPIN_PUSH_SW_1);
  
  // compare the buttonState to its previous state
  if (push_button_1_current_state != push_button_1_last_state)
  {
    // if the state has changed, increment the counter
    if (push_button_1_current_state == HIGH)
    {
      // if the current state is HIGH then the button went from off to on:
      push_button_1_toggle++;
    }
    // Delay a little bit to avoid button bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  push_button_1_last_state = push_button_1_current_state;


  // turns on the push button function & LED via modulo
  int total_options = 3;
  if(push_button_1_toggle % total_options == 1)
  {
    digitalWrite(led_pin_user[0], LOW);
    
    // Enable Dynamixel Torque
    for(uint8_t id = 2; id <= 9; id++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    }
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }

  }
  else if(push_button_1_toggle % total_options == 2)
  {
    gyroControl();
    
  }
  else if(push_button_1_toggle % total_options == 0)
  {
    digitalWrite(led_pin_user[1], HIGH);
    
    digitalWrite(led_pin_user[0], HIGH);

    // Disable Dynamixel Torque
    for(uint8_t id = 2; id <= 9; id++)
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }

    resetInitialGyroArm();
  
  }

  // Read position
  int32_t dxl_present_position = 0;               // Present position

  Serial.print("\n\n\nIMU\n");
    Serial.print("--------");
  Serial.print("\n\tX:");
  Serial.print(IMU.rpy[0]);    // GYRO X
  Serial.print("\n\tY:");
  Serial.print(IMU.rpy[1]);    // GYRO Y
  Serial.print("\n\tZ:");
  Serial.print(IMU.rpy[2]);    // GYRO Z
  Serial.println(" ");
    Serial.println("--------");
    
  for(uint8_t id = 2; id <= 9; id++)
  {
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      Serial.print(packetHandler->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      Serial.print(packetHandler->getRxPacketError(dxl_error));
    }

    Serial.print("[ID:"); Serial.print(id); Serial.print("]");
    Serial.print("\tPresPos:"); Serial.print(dxl_present_position); Serial.print("\t Degree:"); Serial.print(PosToDeg(dxl_present_position)); Serial.print("\n");
  }
  
  Serial.println("--------");

  delay(100);
  
}
