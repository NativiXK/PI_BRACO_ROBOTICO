#include <definitions.h>

// Definição de tipos
typedef struct led_task_parameters_t
{
  gpio_num_t led_gpio;
  TickType_t blink_time;
} led_task_parameters_t;

// Global variables
static led_task_parameters_t led_status = {LED_STATUS, 500};
ServoEasing servos;


// Function prototypes
void Start_Servos(void);
void Start_Tasks(void);
void Start_IOs(void);

// Tasks prototype
void STATUS_TASK(void *pvParameter);
void UPDATE_SERVOS_TASK(void *pvParameter);
void UPDATE_IO_MAP_TASK(void *pvParameter); // TO DO!!!

void setup() {
  // Inicializa fila de movimentos


  // Inicializa todas as entradas e saídas

  // Inicializa todos os servos na biblioteca ServoEasing
  Start_Servos();

  // Inicializa display na biblioteca TFT_esPI

  // Inicializa Tasks
  Start_Tasks();

}

void loop() 
{}

void Start_Tasks(void)
{
  xTaskCreate(
    &STATUS_TASK, // task function
    "STATUS_TASK", // task name
    1024, // stack size in words
    &led_status, // pointer to parameters
    7, // priority
    NULL// out pointer to task handle
  ); 

  xTaskCreate(
    &UPDATE_SERVOS_TASK, // task function
    "UPDATE_SERVOS_TASK", // task name
    4096, // stack size in words
    NULL, // pointer to parameters
    3, // priority
    NULL// out pointer to task handle
  ); 
}

void Start_Servos(void)
{
  // Attach servos
  servos.attach(SERVO_GARRA, START_DEGREE_VALUE);

  // Disable servo update function
  synchronizeAllServosAndStartInterrupt(false);

  // Set overall speed
  setSpeedForAllServos(20);

}

// Task that controls status LED
void STATUS_TASK(void *pvParameter)
{
  // Acessa parâmetros referentes ao status
  gpio_num_t led_gpio = ((led_task_parameters_t *)pvParameter)->led_gpio;
  TickType_t blink_time = ((led_task_parameters_t *)pvParameter)->blink_time;
  uint8_t led_value = 0;
  gpio_reset_pin(led_gpio);
  gpio_set_direction(led_gpio, GPIO_MODE_OUTPUT);

  while (1) {
    gpio_set_level(led_gpio, led_value);
    led_value = !led_value;
    vTaskDelay(blink_time / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

// Esta tarefa vai atualizar a posição dos servos periódicamente
void UPDATE_SERVOS_TASK(void *pvParameter)
{
  TickType_t delay = UPDATE_SERVO_MS;
  bool start_motion = false;

  for (;;)
  {
    // Check if there is a movement in motion buffer
    if ()
    {

    }

    // 
    while(!updateAllServos() && start_motion)
    {
      vTaskDelay(delay);
    }
  }
}