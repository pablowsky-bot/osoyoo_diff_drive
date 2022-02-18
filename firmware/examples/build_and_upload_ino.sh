#!/bin/bash

printf ':::: building ::::\n\n'
arduino-cli compile --fqbn arduino:avr:uno left_encoder_test

printf ':::: uploading ::::\n\n'
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno left_encoder_test
