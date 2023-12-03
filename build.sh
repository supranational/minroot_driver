#!/bin/sh

# Copyright Supranational LLC
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

set -e
set -x

[ -d libft4222 ] || mkdir libft4222

# Fetch the FTDI driver
case `uname -s` in
    Linux)
        if [ ! -f libft4222/libft4222.h ]; then
            wget -O - https://ftdichip.com/wp-content/uploads/2022/06/libft4222-linux-1.4.4.170.tgz \
            | (cd libft4222; tar zxf -)
        fi
        case `uname -p` in
            x86_64)     ftdi_path=libft4222/build-x86_64;;
            aarch64)    ftdi_path=libft4222/build-arm-v8;;
            *)          echo "Usupported platform"; exit 1;;
        esac
        extra_ldflags='-static-libstdc++'
        if (grep -q -e '^flags.*\badx\b' /proc/cpuinfo) 2>/dev/null; then
            CFLAGS="-D__ADX__"
        fi
        ;;
    Darwin)
        if [ ! -f libft4222/libft4222.h ]; then
            curl -sSf https://ftdichip.com/wp-content/uploads/2022/06/LibFT4222-mac-v1.4.4.170.zip \
            | tar xf -
            hdiutil attach libft4222.1.4.4.170.dmg -mountroot . -nobrowse
            rm -f libft4222.1.4.4.170.dmg
            cp -r ft4222/ libft4222/
            hdiutil detach ft4222
        fi
        ftdi_path=libft4222/build
        extra_ldflags='-framework Foundation -framework IOKit'
        if [ -f libft4222/build/libgmp.a ]; then
            CFLAGS="-arch x86_64 -arch arm64"
            CXXFLAGS="-arch x86_64 -arch arm64"
        elif [ `sysctl -n hw.optional.adx 2>/dev/null` = "1" ]; then
            CFLAGS="-D__ADX__"
        fi
        ;;
    *)  echo "Unsupported OS"; exit 1
        ;;
esac

if [ ! -d semolina/src ]; then
    git submodule init
    git submodule update
fi

${CC:-cc} ${CFLAGS} -g -O -c semolina/src/pasta_vdf.c semolina/src/assembly.S
trap 'rm -f pasta_vdf.o assembly.o' 0
rm -f minroot
${CXX:-c++} ${CXXFLAGS} -std=c++11 -pthread -g -O -o minroot -Wall -Wextra \
    -Ilibft4222 -Isrc \
    src/main.cpp src/driver.cpp src/ftdi_driver.cpp \
    pasta_vdf.o assembly.o \
    -L$ftdi_path -lft4222 -lgmp $extra_ldflags
