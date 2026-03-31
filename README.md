# Lizard

Lizard is a domain-specific language to define and control hardware behaviour.
It is intended to run on embedded systems which are connected to motor controllers, sensors etc.
Most of the time it is used in combination with a higher level engine like ROS or RoSys.
You can think of the microcontroller as the machine's lizard brain which ensures basic safety and performs all time-critical actions.

The full documentation is available on https://lizard.dev.

## Getting Started
```
git clone https://github.com/Agroecology-Lab/lizard.git
cd lizard
sudo rm -f sdkconfig
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py set-target esp32s3
echo "CONFIG_COMPILER_CXX_EXCEPTIONS=y" >> sdkconfig.defaults.esp32s3
echo "CONFIG_COMPILER_CXX_EXCEPTIONS=y" >> sdkconfig.defaults.esp32
echo "CONFIG_BT_ENABLED=y" >> sdkconfig.defaults.esp32s3
echo "CONFIG_BT_NIMBLE_ENABLED=y" >> sdkconfig.defaults.esp32s3
git config --global --add safe.directory '*'
pip install -r requirements.txt --break-system-packages
git submodule update --init --recursive

docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 /bin/bash -c "
    idf.py set-target esp32s3 && 
    echo 'CONFIG_BT_ENABLED=y' >> sdkconfig && 
    echo 'CONFIG_BT_NIMBLE_ENABLED=y' >> sdkconfig && 
    idf.py build"

docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 /bin/bash -c "
    echo 'CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y' >> sdkconfig &&
    idf.py build"

```


See also https://lizard.dev/getting_started/
