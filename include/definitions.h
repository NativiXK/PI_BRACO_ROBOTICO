#include <Arduino.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/"
#include "driver/gpio.h"
#include <ServoEasing.hpp>

// Pinout definitions
#define SERVO_GARRA GPIO_NUM_5
#define LED_STATUS GPIO_NUM_2

// RTOS Tasks definitions
#define UPDATE_SERVO_MS 20
#define UPDATE_STATUS_MS 50
#define UPDATE_IO_MAP_MS 10

// Motion definitions
#define START_DEGREE_VALUE  0