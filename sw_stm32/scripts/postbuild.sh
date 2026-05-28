#!/bin/sh

PROJ="/Users/dirk/Documents/GitHub/sw_sensor/sw_stm32"
CONFIG="${1:-Release}"
BUILD="$PROJ/$CONFIG"

cd "$PROJ" || exit 1

.venv/bin/python scripts/create-git-info-header.py || exit 1
.venv/bin/python scripts/pack.py || exit 1

mkdir -p "$BUILD"

TAG=$(git describe --tags --abbrev=0 2>/dev/null)
[ -z "$TAG" ] && TAG="0.0.0"

TAG_SAFE=$(echo "$TAG" | sed 's/^v//' | tr '.' '-')

OUTBASE="larus_sensor_v${TAG_SAFE}-0"

if [ -f "$BUILD/sw_sensor.elf" ]; then
    mv -f "$BUILD/sw_sensor.elf" "$BUILD/${OUTBASE}.elf"
    echo "Erzeugt: $BUILD/${OUTBASE}.elf"
fi

if [ -f "$PROJ/${OUTBASE}.bin" ]; then
    mv -f "$PROJ/${OUTBASE}.bin" "$BUILD/${OUTBASE}.bin"
elif [ -f "$PROJ/sw_sensor.bin" ]; then
    mv -f "$PROJ/sw_sensor.bin" "$BUILD/${OUTBASE}.bin"
elif [ -f "$BUILD/sw_sensor.bin" ]; then
    mv -f "$BUILD/sw_sensor.bin" "$BUILD/${OUTBASE}.bin"
fi

echo "Post-build fertig: ${OUTBASE}"