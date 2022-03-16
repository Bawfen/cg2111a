arduino-cli compile --fqbn arduino:avr:uno Alex.ino
arduino-cli upload --fqbn arduino:avr:uno Alex.ino -p /dev/ttyACM0 --verbose
echo "Done with upload!"

# arduino-cli monitor -p /dev/ttyACM0
