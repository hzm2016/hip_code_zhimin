//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
#include "Serial_Com.h"
#include "Wireless_IMU.h"
#include <Arduino.h>
#include "MovingAverage.h"
/*MOTOR*/
#include <FlexCAN_T4.h>
// #include "Motor_Control_Tmotor.h"
#include "Sig_Motor_Control.h" 

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

/*Filter*/
MovingAverage LTAVx(12);
MovingAverage RTAVx(12);
float f_LTAVx = 0;
float f_RTAVx = 0;

CAN_message_t msgR;

int Motor_ID = 1;
int Motor_ID2 = 2;
int CAN_ID = 3; // Teensy 4.1 CAN bus port

double torque_command = 0;
double velocity_command = 0;
double position_command = 0;

double M1_torque_command = 0;
double M2_torque_command = 0;

int LimitInf = -18;
int LimitSup = 18;

float p_des = 0;
float v_des = 0;
float kp = 0;
float kd = 0;
float t_ff = 0;

Motor_Control_Tmotor sig_m1(0x001, CAN_ID);   
Motor_Control_Tmotor sig_m2(0x000, CAN_ID);    

/*MOTOR*/

/*Isra Serial Class Setup*/
Serial_Com Serial_Com;  

/*Sensors Setup*/
IMU imu;  

/*Serial Send*/  
size_t Send_Length = 11;
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };  

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;
uint16_t R_IMUX_int = 0x00;

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00;

uint16_t L_CMD_int16 = 0x7fff;
float L_CMD_serial = 0.0;

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial = 0.0;

float IMUX_float = 0;
float IMU11 = 0;
float IMU22 = 0;
float IMU33 = 0;
float IMU44 = 0;

/* Time control*/
// unsigned long Delta_T1 = 35;  //Looks like increasing this improves stability, but mkaes the torque less smooth
// unsigned long t_i, t_pr1;
// unsigned long beginning = 0;
double t; 
// double HZ = 1.0 / (Delta_T1 / 1000.0); 


//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
// double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ctrl = 100;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;  // [Hz] Bluetooth sending data frequency
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
char data_rs232_rx[60] = {0};  // Data array for bluetooth data receive:  computer->USB->Adafruit Feather nRF52840 Express(central)->bluetooth->Adafruit Feather nRF52840 Express(peripheral)->RS232->Teensy

int L_leg_IMU_angle = 0;                //left knee angle
int R_leg_IMU = 0;  
int L_motor_torque = 0;
int R_motor_torque = 0;
int L_motor_torque_desired = 0;
int R_motor_torque_desired = 0;
int t_teensy = 0;
int M_Selected = 0;
int CtrlMode_Selected = 0;
int GUI_cmd = 0;
double GUI_K = 0.01;
//**************************

void setup() {
  delay(3000);
  Serial.begin(115200);   //115200/9600=12
  Serial2.begin(115200);  //115200/9600=12
  //Serial7.begin(115200);  // Communication with Raspberry PI or PC for High-lever controllers like RL
  Serial5.begin(115200);  //used to communication with bluetooth peripheral. Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)
  
  Serial_Com.INIT();
  //#################
  Serial.println("SETUP DONE");
  Serial.print("Controller executed at ");
  Serial.print(String(cyclespersec_ctrl));
  Serial.println(" Hz");
  Serial.print("BT com executed at ");
  Serial.print(String(cyclespersec_ble));
  Serial.println(" Hz");
  //####################

  initial_CAN();

  delay(100);

  IMUSetup();
  t_0 = micros();   
}

// //// initial sig motor //// 
// void initial_Sig_motor() {  
//   // sig_m1.error_clear();    
//   // delay(200); 

//   // sig_m1.reboot();  
//   // delay(200);  

//   // sig_m1.sig_motor_reset();   
//   // sig_m2.sig_motor_reset();   
//   // delay(1000);  

//   // // delay(1000);  
//   // sig_m1.sig_encoder_reset();    
//   // sig_m2.sig_encoder_reset();    
//   // delay(10000);   

//   /////////// set control mode /////////  
//   if (ctl_mode == 1)  
//   {
//     sig_m1.sig_mit_ctl_mode_start();      
//     sig_m2.sig_mit_ctl_mode_start();  
//   } 
//   else
//   {
//     sig_m1.sig_torque_ctl_mode_start();    
//     delay(200);   
//     sig_m2.sig_torque_ctl_mode_start();        
//   } 
//   delay(200);  
  
//   sig_m1.sig_motor_start();    
//   sig_m1.request_pos_vel();    
//   delay(500);   

//   sig_m2.sig_motor_start();    
//   sig_m2.request_pos_vel();     
//   delay(500);    

//   if (ctl_mode == 1)  
//   {
//     sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);     
//     sig_m2.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);          
//     receive_mit_ctl_feedback();     
//   }
//   else{
//     // sig_m1.request_torque();   
//     sig_m1.sig_torque_cmd(0.01);    
//     delay(200);    
//     // sig_m2.request_torque();   
//     sig_m2.sig_torque_cmd(0.01);      
//     delay(200);   
//   } 

//   for (int i =0; i < 500; i++)
//   {
//     receive_torque_ctl_feedback();     
//   }
//   // delay(500);  
//   initial_pos_1 = sig_m1.pos;     
//   initial_pos_2 = sig_m2.pos;    

//   delay(500); 

//   /////// command initial setting ///////
//   M1_torque_command = 0.0;    
//   M2_torque_command = 0.0;         
// }

void loop() {

  imu.READ();  
  Serial_Com.READ2();   
  
  current_time = micros() - t_0;  
  t = current_time / 1000000.0;  

  if (current_time - previous_time > Tinterval_ctrl_micros) {
    
    if (current_time - previous_time_ble > Tinterval_ble_micros) {

      Receive_ble_Data();
      Transmit_ble_Data(); // send the BLE data

      previous_time_ble = current_time;
    }

    // fakeIMU();    
    RealIMU();    

    Serial_Com.WRITE(Send, Send_Length);  

    L_CMD_int16 = (Serial_Com.SerialData2[3] << 8) | Serial_Com.SerialData2[4];
    L_CMD_serial = Serial_Com.uint_to_float(L_CMD_int16, -20, 20, 16);    

    R_CMD_int16 = (Serial_Com.SerialData2[5] << 8) | Serial_Com.SerialData2[6];
    R_CMD_serial = Serial_Com.uint_to_float(R_CMD_int16, -20, 20, 16);  

    M1_torque_command = GUI_K * L_CMD_serial;   
    M2_torque_command = GUI_K * R_CMD_serial;   

    // M1_Torque_Control_Example();  
    Wait(1100); // Increasing this increases stability, but less smooth
    // M2_Torque_Control_Example();  
    Wait(1100);  

    previous_time = current_time;
  }
}

void IMUSetup() {
  imu.INIT();
  delay(500);
  imu.INIT_MEAN();
}

void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("Can bus setup done...");
  delay(200);
}

void fakeIMU() {
  IMU11 = 150.0 * sin(t / 5.0);
  IMU22 = 150.0 * cos(t / 5.0);
  IMU33 = 700.0 * sin(t / 5.0);
  IMU44 = 700.0 * cos(t / 5.0);

  L_IMUX_int = Serial_Com.float_to_uint(IMU11, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(IMU22, -180, 180, 16);

  L_IMUV_int = Serial_Com.float_to_uint(IMU33, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(IMU44, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}

void RealIMU() {
  f_LTAVx = LTAVx.addSample(imu.LTAVx);
  f_RTAVx = RTAVx.addSample(imu.RTAVx);

  L_IMUX_int = Serial_Com.float_to_uint(imu.LTx, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(imu.RTx, -180, 180, 16);

  //  L_IMUV_int = Serial_Com.float_to_uint(imu.LTAVx, -800, 800, 16);
  //  R_IMUV_int = Serial_Com.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_IMUV_int = Serial_Com.float_to_uint(f_LTAVx, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(f_RTAVx, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}


void RealIMU_Reset() {
  float reset_imu = 0;

  L_IMUX_int = Serial_Com.float_to_uint(reset_imu, -180, 180, 16);
  R_IMUX_int = Serial_Com.float_to_uint(reset_imu, -180, 180, 16);

  //  L_IMUV_int = Serial_Com.float_to_uint(imu.LTAVx, -800, 800, 16);
  //  R_IMUV_int = Serial_Com.float_to_uint(imu.RTAVx, -800, 800, 16);

  L_IMUV_int = Serial_Com.float_to_uint(reset_imu, -800, 800, 16);
  R_IMUV_int = Serial_Com.float_to_uint(reset_imu, -800, 800, 16);

  Send[0] = 0x31;
  Send[1] = 0x32;
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;
  Send[4] = R_IMUX_int >> 8;
  Send[5] = R_IMUX_int & 0xFF;
  Send[6] = L_IMUV_int >> 8;
  Send[7] = L_IMUV_int & 0xFF;
  Send[8] = R_IMUV_int >> 8;
  Send[9] = R_IMUV_int & 0xFF;
  Send[10] = 0x33;
}

void Wait(unsigned long delay_control) {
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  } while (Time_Control < Time_Delta);
}

//float mAvg(float x){
//
//  /* Begin
//  #include "MovingAverage.h"
//  MovingAverage filter(12);
//  */
//
//  float avg=filter.addSample(x);
//  return avg;
//  }

void Receive_ble_Data(){
  if (Serial5.available() >= 20) {
    // Read the incoming byte:
    Serial5.readBytes(&data_rs232_rx[0], 20);

    if (data_rs232_rx[0] == 165) { // Check the first byte

      if (data_rs232_rx[1] == 90) { // Check the second byte

        if (data_rs232_rx[2] == 20) { // Check the number of elemnts in the package

          M_Selected        = data_rs232_rx[4];
          CtrlMode_Selected = data_rs232_rx[6];
          GUI_cmd           = data_rs232_rx[8];

          GUI_K = GUI_cmd / 100.0;

          Serial.print("| Motor ");
          Serial.print(M_Selected, DEC); // This contains the motor number
          Serial.print(" selected | Control Mode ");
          Serial.print(CtrlMode_Selected, DEC); // This contains the Control mode
          Serial.print(" selected | Command ");
          //Serial.print(GUI_cmd, DEC); // This contains the desired command
          Serial.print(GUI_K);
          Serial.println(" sent |");
        }
      }
    }
  }
}

void Transmit_ble_Data(){
  t_teensy = t * 100;
  L_leg_IMU_angle = imu.LTx * 100;
  R_leg_IMU = imu.RTx * 100; 
  L_motor_torque = 0.0 * 100;
  R_motor_torque = 0.0 * 100;
  L_motor_torque_desired = M1_torque_command *100;
  R_motor_torque_desired = M2_torque_command *100;

  ////*** Totally, we send 32byte data
  // 0    header 165
  // 1    header 90
  // 2    bluetooth data length
  // ...

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = L_leg_IMU_angle;
  data_ble[4]  = L_leg_IMU_angle >> 8;
  data_ble[5]  = R_leg_IMU;
  data_ble[6]  = R_leg_IMU >> 8;
  data_ble[7]  = L_motor_torque;
  data_ble[8]  = L_motor_torque >> 8;
  data_ble[9]  = R_motor_torque;
  data_ble[10] = R_motor_torque >> 8;
  data_ble[11] = L_motor_torque_desired;
  data_ble[12] = L_motor_torque_desired >> 8;
  data_ble[13] = R_motor_torque_desired;
  data_ble[14] = R_motor_torque_desired >> 8;
  data_ble[15] = t_teensy;
  data_ble[16] = t_teensy >> 8;
  data_ble[17] = 0;
  data_ble[18] = 0 >> 8;
  data_ble[19] = 0;
  data_ble[20] = 0;
  data_ble[21] = 0;
  data_ble[22] = 0 >> 8;
  data_ble[23] = 0;
  data_ble[24] = 0 >> 8;
  data_ble[25] = 0;
  data_ble[26] = 0 >> 8;
  data_ble[27] = 0;
  data_ble[28] = 0 >> 8;

  Serial5.write(data_ble, datalength_ble);
  //Serial7.write(data_ble, datalength_ble);
  //Serial.print("Transmit Data Function Executed");
}

double derivative(double dt, double derivative_prev[], double *actual_in_ptr, double *prev_in_ptr){
  int i;
  double diff = 0.0, diff_sum = 0.0;
  if (dt != 0.0){
    for (i = 0; i < 3; i++){
      diff_sum += derivative_prev[i];
    }
    diff = (diff_sum + (*actual_in_ptr - *prev_in_ptr) / dt) / (i + 1);
  } else 
    diff = derivative_prev[3];
  return diff;
}