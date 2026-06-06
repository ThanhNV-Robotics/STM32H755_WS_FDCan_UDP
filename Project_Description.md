### FDCan - UDP on STM32H755 Nucleo
## Overview
Total Number of Motors: 12; 6 motors/leg

Motor ID: 

Left Leg (Joint name: Driver ID)
left_hip_pitch_joint: 10
left_hip_roll_joint: 20
left_hip_yaw_joint: 30
left_knee_pitch_joint: 40
left_ankle_upper_motor: 50
left_ankle_lower_motor: 60

FDCan data bit rate: 5 Mbps
Normal bit rate: 1Mpbs

# Motor Driver - MD80
MD protocol: refer to mab_can_protocol.md

## CM4: for FDcan Communication

mab can protocol lib file: mab_can.h , mab_can.c

# Step 1: ping 5Mbps
Expectation: return: motor id on left leg: 10-60
If no return: on LED_Red (LED_Red_Pin, LED_Red_GPIO_Port)

Scheme 1: no connection to motor, so it should turn on LED_Red

Code to implement (guess) : Maybe fdcan.c file to implement some initialization???