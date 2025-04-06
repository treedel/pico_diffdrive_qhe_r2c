#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#include "defaultPins.h"

#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "servo.h"

#define TOTAL_CHANNELS 2

// Defining pins
const uint ENC_PINS[TOTAL_CHANNELS] = {16, 20};
const uint MOTOR_PINS[TOTAL_CHANNELS][TOTAL_MOTOR_PINS] = {
  {2, 3},
  {4, 5}
};
const uint DOOR_SERVO_PIN = 22;

// For PID loop control
const float PID_LOOP_RATE = 20.0;
const float PID_LOOP_S = 1.0 / PID_LOOP_RATE;
const float PID_LOOP_MS = PID_LOOP_S * 1000;

// LPF Smoothing Factor (0 < alpha < 1)
const float LPF_ALPHA = 0.3;

Motor left_motor, right_motor;
EncoderCounter left_encoder, right_encoder;
PidControl left_pid_controller, right_pid_controller;

// LPF state variables
float left_filtered_speed = 0;
float right_filtered_speed = 0;

// Serial interface variables
#define BUFFER_SIZE 20
char input_buffer[BUFFER_SIZE];

bool pid_control_run(__unused struct repeating_timer *t) {
  update_encoder_values(&left_encoder);
  update_encoder_values(&right_encoder);

  // One motor is oriented in reverse
  int32_t left_encoder_delta = -get_encoder_delta(left_encoder) * PID_LOOP_RATE;
  int32_t right_encoder_delta = get_encoder_delta(right_encoder) * PID_LOOP_RATE;

  // Apply Low-Pass Filter (LPF)
  left_filtered_speed = (LPF_ALPHA * left_encoder_delta) + ((1 - LPF_ALPHA) * left_filtered_speed);
  right_filtered_speed = (LPF_ALPHA * right_encoder_delta) + ((1 - LPF_ALPHA) * right_filtered_speed);

  // Use filtered values in PID calculation
  int32_t left_control_level = calculate_pid_output(&left_pid_controller, left_filtered_speed);
  int32_t right_control_level = calculate_pid_output(&right_pid_controller, right_filtered_speed);

  // Use the PID output to drive the motors
  control_motor(&left_motor, left_control_level);
  control_motor(&right_motor, right_control_level);

  /* printf("Left: %d (Filtered: %.2f) : %d, Right %d (Filtered: %.2f) : %d\n",
         left_encoder_delta, left_filtered_speed, left_control_level,
         right_encoder_delta, right_filtered_speed, right_control_level); */

  return true;
}

// Read input character by character to obtain a string
void getString(char input_buffer[], uint buffer_size) {
  char c;
  uint index = 0;
  while (1) {
      c = getchar();
      if (c == '\n' || c == '\r') break;
      if (index < buffer_size - 1) input_buffer[index++] = c;
  }
  input_buffer[index] = '\0';
}

void ser_interface_run() {
  // Getting string and convert it into float
  char command;
  float arg1;
  float arg2;

  getString(input_buffer, BUFFER_SIZE);
  sscanf(input_buffer, "%c %f %f", &command, &arg1, &arg2);

  switch (command) {
    case 'c':
      set_pid_target(&left_pid_controller, arg1);
      set_pid_target(&right_pid_controller, arg2);
      printf("OK\r\n");
      break;

    case 'e':
      printf("%d %d\r\n", -get_encoder_counts(left_encoder), get_encoder_counts(right_encoder));
      break;

    case 'r':
      watchdog_reboot(0, SRAM_END, 10);
      break;

    case 's':
      set_servo_angle(DOOR_SERVO_PIN, arg1);
      printf("OK\r\n");
      break;

    default:
      printf("Invalid command\r\n");
  }
}

int main() {
    stdio_init_all();

    // Allow time for serial monitor to start
    sleep_ms(2000);

    // Configure servo
    configure_servo(DOOR_SERVO_PIN);

    // Configure encoders
    configure_encoder_counter(&left_encoder, ENC_PINS[0]);
    configure_encoder_counter(&right_encoder, ENC_PINS[1]);
    
    // Configure motors
    configure_motor(&left_motor, MOTOR_PINS[0][0], MOTOR_PINS[0][1]);
    configure_motor(&right_motor, MOTOR_PINS[1][0], MOTOR_PINS[1][1]);

    // Configure PID controllers
    configure_pid_control(&left_pid_controller, PID_LOOP_S, 10, 100, 0);
    configure_pid_control(&right_pid_controller, PID_LOOP_S, 10, 100, 0);

    // Halt the motors initially
    set_pid_target(&left_pid_controller, 0);
    set_pid_target(&right_pid_controller, 0);

    struct repeating_timer timer;
    add_repeating_timer_ms(PID_LOOP_MS, pid_control_run, NULL, &timer);

    while(1) {
      ser_interface_run();
    }
}
