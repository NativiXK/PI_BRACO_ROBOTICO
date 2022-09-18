#include <Arduino.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <ESP32Servo.h>

// Pinout definitions
#define LED_STATUS 2
#define SERVO_J1_PIN 18    // Pino Servo do tronco
#define SERVO_J2_PIN 19    // Pino Servo do braço
#define SERVO_J3_PIN 21    // Pino Servo do cotovelo
#define SERVO_CLAW_PIN 5   // Pino Servo da garra
  // Buttons
#define B1_Start 26        // Botão para iniciar o programa
#define B2_Stop 27         // Botão para parar o programa

#define POT_J1 34          
#define POT_J2 35          
#define POT_J3 32          

// RTOS TASKS DEFINITIONS
#define UPDATE_SERVO_MS 20          // Período de atualização da posição dos servos
#define UPDATE_STATUS_MS 50         // Período de atualização do status
#define UPDATE_IO_MAP_MS 10         // Período de atualização do mapa de IOs
#define UPDATE_CALC_MS 15           // Período de atualização dos cálculos

// MOTION DEFINITION
#define DEGREES_OF_FREEDOM 3        // Graus de liberdade do robo menos a garra
  // Positions
#define START_POS_J1 0              // Posição inicial do servo J1
#define START_POS_J2 0              // Posiçao inicial do servo J2
#define START_POS_J3 0              // Posição inicial do servo j3
#define START_POS_CLAW 180          // Posição inicial da garra (180 - Aberta)

#define MAX_POS_J1 180              // Posição máxima do servo J1
#define MAX_POS_J2 180              // Posição máxima do servo J2
#define MAX_POS_J3 180              // Posição máxima do servo J3
#define MAX_POS_CLAW 180            // Posição máxima da garra

#define MIN_POS_J1 0                // Posição mínima do servo J1
#define MIN_POS_J2 0                // Posição mínima do servo J2
#define MIN_POS_J3 0                // Posição mínima do servo J3
#define MIN_POS_CLAW 0              // Posição mínima do servo J1

  // Velocities
#define VEL_J1 20                   // Velocidade do servo J1 (°/s)
#define VEL_J2 20                   // Velocidade do servo J2 (°/s)
#define VEL_J3 20                   // Velocidade do servo J3 (°/s)
#define VEL_CLAW 20                 // Velocidade da garra (°/s)
