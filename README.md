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
```

```
cat >> sdkconfig <<EOF
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_NIMBLE_MAX_BONDS=15
CONFIG_ZZ_BLE_DEV_PIN=123456
EOF
```

```
echo "CONFIG_ZZ_BLE_DEV_PIN=123456" >> sdkconfig.defaults.esp32s3
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py set-target esp32s3
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py build
# 1. Force the NimBLE and PIN configs into the S3 defaults file
cat >> sdkconfig.defaults.esp32s3 <<EOF
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_NIMBLE_MAX_BONDS=15
CONFIG_ZZ_BLE_DEV_PIN=123456
EOF
# 2. Reset the target to force a merge of these new defaults
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py set-target esp32s3
# 3. Final Build
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py build
```

```
# 1. Force the Bluetooth and PIN settings into the S3 defaults file
cat >> sdkconfig.defaults.esp32s3 <<EOF
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_NIMBLE_MAX_BONDS=15
CONFIG_ZZ_BLE_DEV_PIN=123456
EOF

# 2. Reset the target (this forces the build system to merge the new defaults)
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py set-target esp32s3

# 3. Final Build attempt
docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 idf.py build
```


See https://lizard.dev/getting_started/
