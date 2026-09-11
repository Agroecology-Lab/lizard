# Lizard

Lizard is a domain-specific language to define and control hardware behaviour.
It is intended to run on embedded systems which are connected to motor controllers, sensors, etc.
Most of the time it is used in combination with a higher-level engine like ROS or RoSys.
You can think of the microcontroller as the machine's lizard brain, which ensures basic safety and performs all time-critical actions.

Full documentation: https://lizard.dev.

## Prerequisites

- Docker (runs the ESP-IDF build container, `espressif/idf:v5.3.1`)
- Python 3 + `pip` on the host (for `monitor.py` and `esptool`)
- Board connected via USB — typically enumerates as `/dev/ttyACM0`

## Build

```bash
git clone https://github.com/Agroecology-Lab/lizard.git
cd lizard
sudo rm -f sdkconfig          # drop any sdkconfig left over from a different target
git config --global --add safe.directory '*'
pip install -r requirements.txt --break-system-packages
git submodule update --init --recursive

# Set required config before the first build, so it's picked up when
# sdkconfig is generated from these defaults.
echo "CONFIG_COMPILER_CXX_EXCEPTIONS=y"          >> sdkconfig.defaults.esp32s3
echo "CONFIG_COMPILER_CXX_EXCEPTIONS=y"          >> sdkconfig.defaults.esp32
echo "CONFIG_BT_ENABLED=y"                       >> sdkconfig.defaults.esp32s3
echo "CONFIG_BT_NIMBLE_ENABLED=y"                >> sdkconfig.defaults.esp32s3
echo "CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE=y" >> sdkconfig.defaults.esp32s3
echo "CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y"          >> sdkconfig.defaults.esp32s3
echo 'CONFIG_ESPTOOLPY_FLASHSIZE="8MB"'          >> sdkconfig.defaults.esp32s3

docker run -it --rm -v $(pwd):/project -w /project espressif/idf:v5.3.1 \
  /bin/bash -c "idf.py set-target esp32s3 && idf.py build"
```

To change a `CONFIG_*` value later, edit `sdkconfig.defaults.esp32s3` and re-run `sudo rm -f sdkconfig` before rebuilding, so it regenerates cleanly instead of merging with stale values.

## Flash

With the partition table and flash size above, the app binary lands at `0x20000` — **not** the ESP-IDF default of `0x10000`:

```bash
docker run -it --rm --device=/dev/ttyACM0 -v $(pwd):/project -w /project espressif/idf:v5.3.1 \
  esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 write_flash \
  0x0     build/bootloader/bootloader.bin \
  0x8000  build/partition_table/partition-table.bin \
  0x20000 build/lizard.bin
```

## Monitor

Raw ESP-IDF log output, saved to file:

```bash
docker run -it --rm --device=/dev/ttyACM0 -v $(pwd):/project -w /project espressif/idf:v5.3.1 \
  bash -c "idf.py monitor 2>&1 | tee full_debug.log"
```

Interactive Lizard console — send DSL commands directly, e.g. `core.info()`:

```bash
python3 monitor.py /dev/ttyACM0
```

See also: https://lizard.dev/getting_started/
