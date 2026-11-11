#!/bin/sh
# Copy the C decoder into the STM32 project, which builds on its own: a
# customer can take stm32/ alone, or User/ into a project of theirs.
set -eu
cd "$(dirname "$0")/.."
dest=stm32/hipnuc_f407/User/hipnuc
rm -rf "$dest"
mkdir -p "$dest"
for file in hipnuc_can.c hipnuc_can.h hipnuc_data.c hipnuc_data.h hipnuc_decoder.c hipnuc_decoder.h \
    hipnuc_internal.h; do
    cp "c/hipnuc/$file" "$dest/"
done
