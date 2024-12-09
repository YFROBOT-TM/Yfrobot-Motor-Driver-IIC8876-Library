/*************************************************************
  MotorDriver IIC1508 library Demo
  Creation Date: 2024-06-11
  @ YFROBOT

  This example demonstrates the MotorDriver iic1508's md_iic1508.analogWrite 
  driver 4-way DRV8837 motor driver

  Hardware Hookup:
  MDIIC1508 Breakout --- Arduino
        GND -------------- GND
        5V  --------------  5V
        SDA ------------ SDA (A4)
        SCL ------------ SCL (A5)

*************************************************************/
#include <MDIIC1508.h>
MDIIC1508 md_iic1508;  // Create an MotorDriver iic1508 object to be used throughout

void setup() {
  Serial.begin(115200);
  Serial.println("MotorDriver iic1508 Example");

  // Call md_iic1508.begin(<address>) to initialize the MotorDriver iic1508.
  //  If it successfully communicates, it'll return 1.
  if (md_iic1508.begin(MDIIC1508_ADDRESS) == false) {
    Serial.println("Failed to communicate. Check wiring and address of MotorDriver iic1508 Module.");
    while (1)
      ;  // If we fail to communicate, loop forever.
  }

  Serial.println("Init OK!");
}

void loop() {

  md_iic1508.setMotor(1, 255);  //M1 正转
  delay(1000);
  md_iic1508.stopMotor(1);  //M1 停止
  delay(1000);
  md_iic1508.setMotor(2, 255);  //M2 正转
  delay(1000);
  md_iic1508.setMotor(3, 255);  //M3 正转
  delay(1000);
  md_iic1508.setMotor(4, 255);  //M4 正转
  delay(1000);
  md_iic1508.setMotor(5, 0);  //所有电机停止
  delay(1000);
  md_iic1508.setMotor(5, 255);  //所有电机正转255
  delay(1000);
  md_iic1508.setMotor(5, 0);  //所有电机停止
  delay(1000);
  md_iic1508.setMotor(255, 255, 255, 255);  //正转速度255
  delay(1000);
  md_iic1508.setMotor(100, 100, 100, 100);  //正转速度100
  delay(1000);
  md_iic1508.setMotor(0, 0, 0, 0);  //停止
  delay(1000);
  md_iic1508.setMotor(-255, -255, -255, -255);  //正转速度255
  delay(1000);
  md_iic1508.setMotor(-100, -100, -100, -100);  //反转速度100
  delay(1000);
  md_iic1508.setMotor(0, 0, 0, 0);  //停止
  delay(1000);
}
