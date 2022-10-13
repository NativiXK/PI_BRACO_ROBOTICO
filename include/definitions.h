// Utilities
#define False 0
#define True 1

// Pinout definitions
#define LED_STATUS 2
#define SERVO_J1_PIN 18    // Pino Servo do tronco
#define SERVO_J2_PIN 19    // Pino Servo do braço
#define SERVO_J3_PIN 21    // Pino Servo do cotovelo
#define CLAW_PIN 5         // Pino da garra

  // Buttons
#define B1_Start 26        // Botão para iniciar o programa
#define B2_Stop 27         // Botão para parar o programa
#define B3_Close_Claw 14   // Botão para fechar a garra
#define B4_Open_Claw 12    // Botão para abrir a garra

#define POT_J1 34          
#define POT_J2 35          
#define POT_J3 32          

// RTOS TASKS DEFINITIONS
#define UPDATE_SERVO_MS 20          // Período de atualização da posição dos servos
#define UPDATE_STATUS_MS 50         // Período de atualização do status
#define UPDATE_IO_MAP_MS 10         // Período de atualização do mapa de IOs
#define UPDATE_CALC_MS 15           // Período de atualização dos cálculos

// MOTION DEFINITION
#define ENABLE_EASE_CUBIC
#define DEGREES_OF_FREEDOM 3        // Graus de liberdade do robo menos a garra
#define INITIAL_OP_MODE_J1 0        // 0 = MOVE_INSTANT | 1 = MOVE_EASING | 2 = DISABLED
#define INITIAL_OP_MODE_J2 0        // 0 = MOVE_INSTANT | 1 = MOVE_EASING | 2 = DISABLED
#define INITIAL_OP_MODE_J3 0        // 0 = MOVE_INSTANT | 1 = MOVE_EASING | 2 = DISABLED

  // Positions
#define START_POS_J1 90              // Posição inicial do servo J1
#define START_POS_J2 90              // Posiçao inicial do servo J2
#define START_POS_J3 90              // Posição inicial do servo j3

#define MAX_POS_J1 180              // Posição máxima do servo J1
#define MAX_POS_J2 180              // Posição máxima do servo J2
#define MAX_POS_J3 180              // Posição máxima do servo J3

#define MIN_POS_J1 0                // Posição mínima do servo J1
#define MIN_POS_J2 0                // Posição mínima do servo J2
#define MIN_POS_J3 0                // Posição mínima do servo J3

  // Velocities
#define VEL_J1 50                   // Velocidade do servo J1 (°/s)
#define VEL_J2 50                   // Velocidade do servo J2 (°/s)
#define VEL_J3 50                   // Velocidade do servo J3 (°/s)
