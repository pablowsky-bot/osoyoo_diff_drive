#!/bin/bash

printf ':::: building hello_world_ros ::::\n\n'
arduino-cli compile --fqbn arduino:avr:uno hello_world_ros

printf ':::: building move_motor ::::\n\n'
arduino-cli compile --fqbn arduino:avr:uno move_motor

printf ':::: building read_econder ::::\n\n'
arduino-cli compile --fqbn arduino:avr:uno read_econder
