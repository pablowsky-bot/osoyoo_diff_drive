upload arduino code to jetbot using command line interface:

# set read/write permissions to arduino board
sudo chmod a+rw /dev/ttyACM0

# build and upload sketch (replace hello_world_ros with the example of your choice)
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno hello_world_ros
