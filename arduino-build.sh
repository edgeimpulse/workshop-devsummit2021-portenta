#!/bin/bash
set -e

PROJECT=workshop-devsummit2021-portenta
BOARD=arduino:mbed_portenta:envie_m7:split=100_0
COMMAND=$1
if [ -z "$ARDUINO_CLI" ]; then
    ARDUINO_CLI=$(which arduino-cli || true)
fi
DIRNAME="$(basename "$SCRIPTPATH")"
EXPECTED_CLI_MAJOR=0
EXPECTED_CLI_MINOR=13

CLI_MAJOR=$($ARDUINO_CLI version | cut -d. -f1 | rev | cut -d ' '  -f1)
CLI_MINOR=$($ARDUINO_CLI version | cut -d. -f2)
CLI_REV=$($ARDUINO_CLI version | cut -d. -f3 | cut -d ' '  -f1)

check_dependency()
{
    if [ ! -x "$ARDUINO_CLI" ]; then
        echo "Cannot find 'arduino-cli' in your PATH. Install the Arduino CLI before you continue."
        echo "Installation instructions: https://arduino.github.io/arduino-cli/latest/"
        exit 1
    fi

    if (( CLI_MINOR < EXPECTED_CLI_MINOR)); then
        echo "You need to upgrade your Arduino CLI version (now: $CLI_MAJOR.$CLI_MINOR.$CLI_REV, but required: $EXPECTED_CLI_MAJOR.$EXPECTED_CLI_MINOR.x or higher)"
        echo "See https://arduino.github.io/arduino-cli/installation/ for upgrade instructions"
        exit 1
    fi

    if (( CLI_MAJOR != EXPECTED_CLI_MAJOR || CLI_MINOR != EXPECTED_CLI_MINOR )); then
        echo "You're using an untested version of Arduino CLI, this might cause issues (found: $CLI_MAJOR.$CLI_MINOR.$CLI_REV, expected: $EXPECTED_CLI_MAJOR.$EXPECTED_CLI_MINOR.x)"
    fi

    echo "Installing dependencies..."
    $ARDUINO_CLI core update-index

    echo "Installing Arduino Mbed core..."
    $ARDUINO_CLI core install arduino:mbed_portenta@2.5.2
    echo "Installing Arduino Mbed core OK"

    echo "Installing MKRWAN library..."
    $ARDUINO_CLI lib install MKRWAN@1.0.13  # MKRWAN library
    echo "Installing MKRWAN library OK"
}

get_data_dir() {
	local OUTPUT=$($ARDUINO_CLI config dump | grep 'data: ')
	local lib="${OUTPUT:8}"
	echo "$lib"
}

patch-mbed-2-5-2()
{
    echo "recipe.hooks.linking.prelink.1.pattern=\"{compiler.path}{compiler.c.elf.cmd}\" -E -P -x c {build.extra_ldflags} \"{build.variant.path}/{build.ldscript}\" -o {build.path}/{build.ldscript}" > ${ARDUINO_DATA_DIR}/packages/arduino/hardware/mbed_portenta/2.5.2/platform.local.txt
}

# CLI v0.14 updates the name of this to --build-property
if ((CLI_MAJOR >= 0 && CLI_MINOR >= 14)); then
    BUILD_PROPERTIES_FLAG=--build-property
else
    BUILD_PROPERTIES_FLAG=--build-properties
fi

ARDUINO_DATA_DIR="$(get_data_dir)"
if [ -z "$ARDUINO_DATA_DIR" ]; then
    echo "Arduino data directory not found"
    exit 1
fi

INCLUDE="-I./src"
INCLUDE+=" -I./src/model-parameters"
INCLUDE+=" -I./src/repl"
INCLUDE+=" -I./src/ingestion-sdk-c/"
INCLUDE+=" -I./src/ingestion-sdk-c/inc"
INCLUDE+=" -I./src/ingestion-sdk-c/inc/signing"
INCLUDE+=" -I./src/ingestion-sdk-platform/portenta-h7"
INCLUDE+=" -I./src/sensors"
INCLUDE+=" -I./src/QCBOR/inc"
INCLUDE+=" -I./src/QCBOR/src"
INCLUDE+=" -I./src/mbedtls_hmac_sha256_sw/"
INCLUDE+=" -I./src/edge-impulse-sdk/"

FLAGS="-DARDUINOSTL_M_H"
FLAGS+=" -DMBED_HEAP_STATS_ENABLED=1"
FLAGS+=" -DMBED_STACK_STATS_ENABLED=1"
FLAGS+=" -O3"
FLAGS+=" -g3"
FLAGS+=" -DEI_SENSOR_AQ_STREAM=FILE"
FLAGS+=" -DEIDSP_QUANTIZE_FILTERBANK=0"
FLAGS+=" -DEI_CLASSIFIER_SLICES_PER_MODEL_WINDOW=3"
FLAGS+=" -DEI_DSP_IMAGE_BUFFER_STATIC_SIZE=128"
FLAGS+=" -DEI_CAMERA_FRAME_BUFFER_SDRAM"
#FLAGS+=" -DEI_CAMERA_FRAME_BUFFER_HEAP"
#FLAGS+=" -mfpu=fpv4-sp-d16"
FLAGS+=" -w"

if [ "$COMMAND" = "--build" ];
then
    echo "Building $PROJECT"
    check_dependency
    patch-mbed-2-5-2
    $ARDUINO_CLI compile --fqbn  $BOARD $BUILD_PROPERTIES_FLAG build.extra_flags="$INCLUDE $FLAGS" --output-dir . &
    pid=$! # Process Id of the previous running command
    while kill -0 $pid 2>/dev/null
    do
        echo "Still building..."
        sleep 2
    done
    wait $pid
    ret=$?
    if [ $ret -eq 0 ]; then
        echo "Building $PROJECT done"
    else
        exit "Building $PROJECT failed"
    fi
elif [ "$COMMAND" = "--flash" ];
then
    $ARDUINO_CLI upload -p $($ARDUINO_CLI board list | grep Arduino | cut -d ' ' -f1) --fqbn $BOARD --input-dir .
elif [ "$COMMAND" = "--all" ];
then
    $ARDUINO_CLI compile --fqbn  $BOARD $BUILD_PROPERTIES_FLAG build.extra_flags="$INCLUDE $FLAGS"
    status=$?
    [ $status -eq 0 ] && $ARDUINO_CLI upload -p $($ARDUINO_CLI board list | grep Arduino | cut -d ' ' -f1) --fqbn $BOARD --input-dir .
else
    echo "Nothing to do for target"
fi
