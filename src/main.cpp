#include <definitions.h>

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
  Servo Motor;

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

  // Inicializa todos os servos na biblioteca ServoEasing
  Start_Servos();

  // Inicializa display na biblioteca TFT_esPI

  // Inicializa Tasks
  Start_Tasks();

}

void loop() {}

void Start_Servos(void)
{
  // Initialize Motion gear
  Servos[0] = {0, 0, VEL_J1, MAX_POS_J1, MIN_POS_J1, START_POS_J1, SERVO_J1_PIN};
  Servos[1] = {0, 0, VEL_J2, MAX_POS_J2, MIN_POS_J2, START_POS_J2, SERVO_J2_PIN};
  Servos[2] = {0, 0, VEL_J3, MAX_POS_J3, MIN_POS_J3, START_POS_J3, SERVO_J3_PIN};
  Servos[3] = {0, 0, VEL_CLAW, MAX_POS_CLAW, MIN_POS_CLAW, START_POS_CLAW, SERVO_CLAW_PIN};

  // Attach servos and move them to start position
  for (int i = 0; i < DEGREES_OF_FREEDOM + 1; i++)
  {
    MotionGear *joint = &Servos[i];
    joint->Motor.attach(joint->Pin, map(joint->Min_Pos, 0, 180, 544, 2400), map(joint->Max_Pos, 0, 180, 544, 2400));
    joint->Motor.write(joint->Start_Pos);
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

  // Create a dedicated task to update each servo
  for (int i = 0; i <= DEGREES_OF_FREEDOM + 1; i++)
  {

    xTaskCreate(
      &UPDATE_SERVO_TASK, // task function
      "SERVO_TASK", // task name
      1024, // stack size in words
      &Servos[i], // pointer to parameters
      3, // priority
      NULL// out pointer to task handle
    );
  }

}

void Start_IOs(void)
{
  

}

// This task will receive a MotionGear parameter with all parameters related to the joint
// and update the servo position till reach MoveTo position
void UPDATE_SERVO_TASK(void *ServoParameters)
{
  for(;;) // Forever loop
  {

    MotionGear *joint = (MotionGear *) ServoParameters;

    // Serial.println("Update servo " + String(joint->Pin));

    int pos_diff = abs(joint->MoveTo - joint->Actual);

    // Check if needed to move
    if (pos_diff > 1)
    {
      int diff = (joint->MoveTo - joint->Actual);

      /* Calculate step to update servo, using easing function from servoEasing lib
          Velocity = °/s -> °/1000mS
          Position = °
          Update_interval = 20mS
          

      */

      int step = diff > 0 ? 1: -1;

      joint->Actual = joint->Actual + step;
      joint->Motor.write(joint->Actual);
    }

    vTaskDelay(UPDATE_SERVO_MS / portTICK_PERIOD_MS);
  }
}

void UPDATE_IO_MAP_TASK(void *pvParameter)
{
  for (;;)
  {
    IOMirror.AI_INPUT_1 = map(analogRead(POT_J1), 0, 4095, 0, 180);
    Serial.print("J1: ");
    Serial.print(IOMirror.AI_INPUT_1);
    IOMirror.AI_INPUT_2 = map(analogRead(POT_J2), 0, 4095, 0, 180);
    Serial.print("\tJ2: ");
    Serial.print(IOMirror.AI_INPUT_2);
    IOMirror.AI_INPUT_3 = map(analogRead(POT_J3), 0, 4095, 0, 180);
    Serial.print("\tJ3: ");
    Serial.println(IOMirror.AI_INPUT_3);

    vTaskDelay(UPDATE_IO_MAP_MS / portTICK_PERIOD_MS);
  }
}

void CALCULATE_TASK(void *pvParameter)
{
  for (;;)
  {
      Servos[0].MoveTo = IOMirror.AI_INPUT_1;
      Servos[1].MoveTo = IOMirror.AI_INPUT_2;
      Servos[2].MoveTo = IOMirror.AI_INPUT_3;

  }
}