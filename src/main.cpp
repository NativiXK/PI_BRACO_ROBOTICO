#include <Arduino.h>
#include <definitions.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <ESP32Servo.h>
#include "ServoEasing.hpp"

// Types definitions
typedef struct led_task_parameters_t
{
  byte led_gpio;
  TickType_t blink_time;

} led_task_parameters_t;

typedef struct MotionGear
{
  byte Actual;                // Posição atual
  byte MoveTo;                // Posição final desejada
  byte Velocity;              // Velocidade de deslocamento da junta
  byte Max_Pos;               // Posição máxima do servo
  byte Min_Pos;               // Posição miníma do servo
  byte Start_Pos;             // Posição inicial do servo
  byte Pin;                   // Pino de conexão do servo
  ServoEasing Motor;
  char Name[5];              // Nome do eixo

} MotionGear;

// Type to store 8 digital inputs
typedef struct ios
{
  byte IO_1 : 1;
  byte IO_2 : 2;
  byte IO_3 : 3;
  byte IO_4 : 4;
  byte IO_5 : 5;
  byte IO_6 : 6;
  byte IO_7 : 7;
  byte IO_8 : 8;

} ios;

// Type to store IO reading
typedef struct IOMemory
{
  int AI_INPUT_1;
  int AI_INPUT_2;
  int AI_INPUT_3;
  ios IN;
  ios OUT;
  
} IOMemory;

// Global variables
static led_task_parameters_t led_status = {LED_STATUS, 500};


/*
  MotionGear
  Servos[0] -> Tronco J1
  Servos[1] -> Braco J2
  Servos[2] -> Cotovelo J3
  Servos[3] -> Garra
*/
MotionGear Servos[DEGREES_OF_FREEDOM + 1];

/*
  IO Mapping
  AI - Store the reading from an analog input
  IN  - Store the inputs value
  OUT - Store the outputs value
*/ IOMemory IOMirror;

// Function prototypes
void Start_Servos(void);
void Start_Tasks(void);
void Start_IOs(void);

// Tasks prototype
void STATUS_TASK(void *pvParameter);
void UPDATE_SERVO_TASK(void *ServoParameters);
void UPDATE_IO_MAP_TASK(void *pvParameter);
void CALCULATE_TASK(void *pvParameter);

void setup() {
  Serial.begin(115200);
  // Inicializa fila de movimentos

  // Inicializa todas as entradas e saídas
  Start_IOs();
  // Inicializa todos os servos na biblioteca ServoEasing
  Start_Servos();
  // Inicializa display na biblioteca TFT_esPI

  // Inicializa Tasks
  Start_Tasks();

}

void loop() {}

void Start_IOs(void)
{
  pinMode(B1_Start, INPUT);
  pinMode(B2_Stop, INPUT);
  pinMode(B3_Close_Claw, INPUT);
  pinMode(B4_Open_Claw, INPUT);

}

void Start_Servos(void)
{
  // Initialize Motion gear
  Servos[0] = {0, 0, VEL_J1, MAX_POS_J1, MIN_POS_J1, START_POS_J1, SERVO_J1_PIN};
  Servos[1] = {0, 0, VEL_J2, MAX_POS_J2, MIN_POS_J2, START_POS_J2, SERVO_J2_PIN};
  Servos[2] = {0, 0, VEL_J3, MAX_POS_J3, MIN_POS_J3, START_POS_J3, SERVO_J3_PIN};
  Servos[3] = {0, 0, VEL_CLAW, MAX_POS_CLAW, MIN_POS_CLAW, START_POS_CLAW, SERVO_CLAW_PIN};

  strcpy(Servos[0].Name, "J1");
  strcpy(Servos[1].Name, "J2");
  strcpy(Servos[2].Name, "J3");
  strcpy(Servos[3].Name, "CLAW");

  // Attach servos and move them to start position
  for (int i = 0; i < DEGREES_OF_FREEDOM + 1; i++)
  {
    MotionGear *joint = &Servos[i];
    joint->Motor.setEasingType(EASE_CUBIC_IN_OUT);
    joint->Motor.setSpeed(joint->Velocity);
    joint->Motor.attach(joint->Pin, joint->Start_Pos);
  }

}

void Start_Tasks(void)
{
  // xTaskCreate(
  //   &STATUS_TASK, // task function
  //   "STATUS_TASK", // task name
  //   1024, // stack size in words
  //   &led_status, // pointer to parameters
  //   7, // priority
  //   NULL// out pointer to task handle
  // ); 

  xTaskCreate(
    &UPDATE_IO_MAP_TASK, // task function
    "IO_TASK", // task name
    1024, // stack size in words
    NULL, // pointer to parameters
    5, // priority
    NULL// out pointer to task handle
  );

  xTaskCreate(
    &CALCULATE_TASK, // task function
    "CALCULATE_TASK", // task name
    1024, // stack size in words
    NULL, // pointer to parameters
    5, // priority
    NULL// out pointer to task handle
  );

  //Create a dedicated task to update each servo
  for (int i = 0; i < DEGREES_OF_FREEDOM + 1; i++)
  {

    xTaskCreate(
      &UPDATE_SERVO_TASK, // task function
      strcat(Servos[i].Name, " SERVO_TASK"), // task name
      1024, // stack size in words
      &Servos[i], // pointer to parameters
      4, // priority
      NULL// out pointer to task handle
    );

  }

}

// This task will receive a MotionGear parameter with all parameters related to the joint
// and update the servo position till reach MoveTo position
void UPDATE_SERVO_TASK(void *ServoParameters)
{
  for(;;) // Forever loop
  {

    MotionGear *joint = (MotionGear *) ServoParameters;

    if (joint->Actual != joint->MoveTo)
    {
      joint->Motor.startEaseTo(joint->MoveTo, joint->Velocity, DO_NOT_START_UPDATE_BY_INTERRUPT);
    }
    
    do
    {
      // Serial.println(strcat("UPDATE ", joint->Name));
      vTaskDelay(UPDATE_SERVO_MS / portTICK_PERIOD_MS);
    } while(!joint->Motor.update());
   
    if (joint->Motor.isMoving())
    {
      joint->Actual = joint->MoveTo;
    }

  }
}

void UPDATE_IO_MAP_TASK(void *pvParameter)
{
  for (;;)
  {
    IOMirror.AI_INPUT_1 = map(analogRead(POT_J1), 0, 4095, 0, 180);
    // Serial.print("J1: ");
    // Serial.print(IOMirror.AI_INPUT_1);
    IOMirror.AI_INPUT_2 = map(analogRead(POT_J2), 0, 4095, 0, 180);
    // Serial.print("\tJ2: ");
    // Serial.print(IOMirror.AI_INPUT_2);
    IOMirror.AI_INPUT_3 = map(analogRead(POT_J3), 0, 4095, 0, 180);
    // Serial.print("\tJ3: ");
    // Serial.println(IOMirror.AI_INPUT_3);

    IOMirror.IN.IO_1 = digitalRead(B1_Start);
    IOMirror.IN.IO_2 = digitalRead(B2_Stop);
    IOMirror.IN.IO_3 = digitalRead(B3_Close_Claw);
    IOMirror.IN.IO_4 = digitalRead(B4_Open_Claw);

    vTaskDelay(UPDATE_IO_MAP_MS / portTICK_PERIOD_MS);
  }
}

void CALCULATE_TASK(void *pvParameter)
{
  for (;;)
  {
      // Serial.println("Calculate");
      Servos[0].MoveTo = IOMirror.AI_INPUT_1;
      Servos[1].MoveTo = IOMirror.AI_INPUT_2;
      Servos[2].MoveTo = IOMirror.AI_INPUT_3;

      // Verifica comando para fechar garra
      if (IOMirror.IN.IO_3)
      {
        Serial.println("Close");
        Servos[3].MoveTo = 0;
      }
      // Verifica comando para abrir a garra
      if (IOMirror.IN.IO_4)
      {
        Serial.println("Open");
        Servos[3].MoveTo = 180;
      }

      vTaskDelay(UPDATE_CALC_MS / portTICK_PERIOD_MS);
  }
}