/*
  Simple_collision_avoider.ino - Usage of the libraries Example
  Using the Mona_ESP library in C style.
  Created by Bart Garcia, December 2020.
  bart.garcia.nathan@gmail.com
  Released into the public domain.
*/
//Include the Mona_ESP library
#include <Wire.h>
#include "Mona_ESP_lib.h"


// Include the Mona_ESP library
// Right: 85.71, Left: 85.71 RPM 
// #include <Wire.h>
// #include "Mona_ESP_lib.h"
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 

// Start
// using Wifi and UDP
#include <WiFi.h>
#include <WiFiUdp.h>

#include <esp_now.h>

#define PACKET_SIZE 1460 // Can increase this with kconfig
#define UDP_PORT 54007
#define SERIAL_USB_BAUD 1000000

ESP32Encoder right_encoder;
ESP32Encoder left_encoder;

uint8_t centerHubAddress[] = {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x6c};
//centralhub mac: 8c:ce:4e:bb:4c:6c

// WiFi network name and password:
const char* networkName = "MSc_IoT"; // replace with your network id
const char* networkPswd = "MSc_IoT@UCL"; // replace with your network password

// IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const int udpPort = UDP_PORT;

// Are we currently connected?
boolean connected = false;

// The UDP library class
WiFiUDP udp;
unsigned char packet[PACKET_SIZE];

int right_new_pos, right_old_pos, left_new_pos, left_old_pos;

float theta_d = 0.0, theta_f = 0.0;
float right_vel = 0.0, left_vel = 0.0, right_ref_vel = 0.0, left_ref_vel = 0.0;

char Bot_Id[32];
float X = 0.0, Y = 0.0, Z = 0.0, Orientation = 0.0, X_d = 0.0, Y_d = 0.0, Z_d = 0.0;
bool done = false;


// set sample time
float Ts = 0.01;
float start_time, current_time, elapsed_time;
const float rad2deg = 57.27;
const float deg2rad = -0.01745328;

// Wheel radius is 15mm, every revolute is 3500 pulse
const float robot_radius = 40;
const float wheel_radius = 15;
const float scale_encoder = 0.0043;    // 15/3500
const float pi = 3.14159; 


// initialise variables
float err = 0;
float control_theta=0, control_lin=0; 
float control_R=0, control_L=0, control = 0; 
float intError = 0;
bool read_flag = true;

// Sensor and State Variables
bool IR_values[5] = {false, false, false, false, false}; // IR sensor values
int threshold = 35; // IR sensor detection threshold
int state, old_state;

typedef struct struct_message {
  char ID[32];              // 标识符
  float position[3];   // 位置，包含X, Y, Z坐标
  float headingY;// Y方向的朝向
  float position_d[3];  //目标地址    
  bool IR_states[5];      // IR sensor states
} struct_message;

struct_message myData;

bool receive_flag = false;

bool phaseTwo = false; // flag for phase control

float stopThreshold = 20; 
// end


//Variables
// bool IR_values[5] = {false, false, false, false, false};
//Threshold value used to determine a detection on the IR sensors.
//Reduce the value for a earlier detection, increase it if there
//false detections.
// int threshold = 35;
//State Machine Variable
// 0 -move forward , 1 - forward obstacle , 2 - right proximity , 3 - left proximity
// int state, old_state;

void setup()
{
	//Initialize the MonaV2 robot
	Mona_ESP_init();
  //Initialize variables
  state = 0;
  old_state = 0;

  // Mona_ESP_init();
  Serial.begin(1000000);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  // esp_now_register_send_cb(OnDataSent);
}

//void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t *incomingData, int len) {
void OnDataRecv(const uint8_t *mac,const uint8_t *incomingData, int len) {
    if (len == sizeof(struct_message)) {
        memcpy(&myData, incomingData, sizeof(myData));
        receive_flag=true;

      strcpy(Bot_Id, myData.ID);
      X=myData.position[0];
      Y=myData.position[1];
      Z=myData.position[2];
      Orientation= myData.headingY*deg2rad;
      // Orientation=normalizeAngle(myData.headingY*deg2rad);
      X_d=myData.position_d[0];
      Y_d=myData.position_d[1];
      Z_d=myData.position_d[2];
    } else {
        Serial.println("Received data size does not match!");
    }
}

void loop(){

      Serial.print("Bot_Id:");
      Serial.print(Bot_Id);
      Serial.print(" ");

      Serial.print("X: ");
      Serial.print(X);
      Serial.print(" ");

      Serial.print("Y: ");
      Serial.print(Y);
      Serial.print(" ");

      Serial.print("Z: ");
      Serial.print(Z);
      Serial.print(" ");

      Serial.print("Orientation:");
      Serial.print(Orientation);
      Serial.print(" ");

      Serial.print("X_d:");
      Serial.print(X_d);
      Serial.print(" ");

      Serial.print("Y_d:");
      Serial.print(Y_d);
      Serial.print(" ");

      Serial.print("Z_d:");
      Serial.println(Z_d);
      Serial.print(" ");
  //--------------Motors------------------------
  //Set motors movement based on the state machine value.
  if(state == 0){
    // Start moving Forward
    Motors_forward(150);
  }
  if(state == 1){
    //Spin to the left
    Motors_spin_left(100);
  }
    if(state == 2){
    //Spin to the left
    Motors_spin_left(100);
  }
    if(state == 3){
    //Spin to the right
    Motors_spin_right(100);
  }

  //--------------IR sensors------------------------
  //Decide future state:
	//Read IR values to determine maze walls
  IR_values[0] = Detect_object(1,threshold);
  IR_values[1] = Detect_object(2,threshold);
  IR_values[2] = Detect_object(3,threshold);
  IR_values[3] = Detect_object(4,threshold);
  IR_values[4] = Detect_object(5,threshold);

	//--------------State Machine------------------------
	//Use the retrieved IR values to set state
	//Check for frontal wall, which has priority
	if(IR_values[2] or IR_values[3] or IR_values[4]){
		state=1;
	}
	else if(IR_values[0]){ //Check for left proximity
		state=3;
	}
	else if(IR_values[4]){// Check for right proximity
		state=2;
	}
	else{ //If there are no proximities, move forward
		state=0;
	}

	delay(5);
}
