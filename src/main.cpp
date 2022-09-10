#include <definitions.h>

// Types definitions
typedef struct led_task_parameters_t
{
  gpio_num_t led_gpio;
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
};

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

// Function prototypes
void Start_Servos(void);
void Start_Tasks(void);
void Start_IOs(void);

// Tasks prototype
void STATUS_TASK(void *pvParameter);
void UPDATE_SERVO_TASK(void *MotionGear);
void UPDATE_IO_MAP_TASK(void *pvParameter); // TO DO!!!

void setup() {

  // Inicializa fila de movimentos

  // Inicializa todas as entradas e saídas

  // Inicializa todos os servos na biblioteca ServoEasing
  Start_Servos();

  // Inicializa display na biblioteca TFT_esPI

  // Inicializa Tasks
  // Start_Tasks();

}

void loop() {}

void Start_Servos(void)
{
  // Initialize Motion gear
  Servos[0] = {0, 0, VEL_J1, MAX_POS_J1, MIN_POS_J1, START_POS_J1, SERVO_J1_PIN};
  Servos[1] = {0, 0, VEL_J2, MAX_POS_J2, MIN_POS_J2, START_POS_J2, SERVO_J2_PIN};
  Servos[2] = {0, 0, VEL_J3, MAX_POS_J3, MIN_POS_J3, START_POS_J3, SERVO_J3_PIN};
  Servos[3] = {0, 0, VEL_CLAW, MAX_POS_CLAW, MIN_POS_CLAW, START_POS_CLAW, SERVO_CLAW_PIN};

  for (int i = 0; i <= DEGREES_OF_FREEDOM + 1; i++)
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

  // Create a dedicated task to update servos
  for (int i = 0; i <= DEGREES_OF_FREEDOM + 1; i++)
  {
    xTaskCreate(
    &UPDATE_SERVO_TASK, // task function
    "SERVO_TASK", // task name
    2048, // stack size in words
    Servos[i], // pointer to parameters
    3, // priority
    NULL// out pointer to task handle
    );
  }

}

// Task that controls status LED
// void STATUS_TASK(void *pvParameter)
// {
//   // Acessa parâmetros referentes ao status
//   gpio_num_t led_gpio = ((led_task_parameters_t *)pvParameter)->led_gpio;
//   TickType_t blink_time = ((led_task_parameters_t *)pvParameter)->blink_time;
//   uint8_t led_value = 0;
//   gpio_reset_pin(led_gpio);
//   gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);

//   while (1) {
//     gpio_set_level(led_gpio, led_value);
//     led_value = !led_value;
//     vTaskDelay(blink_time / portTICK_PERIOD_MS);
//   }
//   vTaskDelete( NULL );
// }

// This task will receive a MotionGear parameter with all parameters related to the joint
// and update the servo position till reach MoveTo position
void UPDATE_SERVO_TASK(void *MotionGear)
{
  for(;;) // Forever loop
  {
    MotionGear joint = ((MotionGear *)MotionGear);

    // Check if needed to move
    if (joint->Actual != joint->MoveTo)
    {
      // Calculate step to update servo, using easing function from servoEasing lib
      int step = (joint->MoveTo - joint-.Actual) > 0 ? (joint->MoveTo - joint-.Actual) /  : ;
      joint->Motor.write()
    }

    vTaskDelay(UPDATE_SERVO_MS / portTICK_PERIOD_MS);
  }
}
