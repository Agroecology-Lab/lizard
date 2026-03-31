# Lizard

Lizard is a domain-specific language to define and control hardware behaviour.
It is intended to run on embedded systems which are connected to motor controllers, sensors etc.
Most of the time it is used in combination with a higher level engine like ROS or RoSys.
You can think of the microcontroller as the machine's lizard brain which ensures basic safety and performs all time-critical actions.

The full documentation is available on https://lizard.dev.

## Getting Started

git clone https://github.com/Agroecology-Lab/lizard.git
cd lizard
pip install -r requirements.txt --break-system-packages
./compile.sh esp32s3


See https://lizard.dev/getting_started/
