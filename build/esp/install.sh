#!/bin/bash

esp32c3_package_list="git wget flex bison gperf python3 python3-setuptools cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0"
dpkg -s ${esp32c3_package_list} &> /dev/null
if [ $? -ne 0 ];then
    echo "Install software package"
    sudo apt-get install -y ${esp32c3_package_list}
    if [ $? -ne 0 ];then
        echo "software package install failed."
        exit 1
    fi
else
    echo "Software package already installed"
fi

esp32c3_gcc_filename="riscv32-esp-elf-gcc11_2_0-esp-2022r1-linux-amd64.tar.xz"
esp32c3_gcc_hash="52710f804df4a033a2b621cc16cfa21023b42052819a51e35a2a164140bbf665 ${esp32c3_gcc_filename}"
script_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

riscv32-esp-elf-gcc -v &> /dev/null

if [ $? -ne 0 ] && [ ! -f "${HOME}/.local/riscv32-esp-elf/bin/riscv32-esp-elf-gcc" ];then
    echo "Compiler does not exist, start download..."
    if [ ! -f ${esp32c3_gcc_filename} ];then
        wget https://dl.espressif.com/github_assets/espressif/crosstool-NG/releases/download/esp-2022r1/${esp32c3_gcc_filename}
    else
        echo $esp32c3_gcc_hash | sha256sum --check
        if [ $? -ne 0 ];then
            rm -f ${esp32c3_gcc_filename}
            wget https://dl.espressif.com/github_assets/espressif/crosstool-NG/releases/download/esp-2022r1/${esp32c3_gcc_filename}
        fi
    fi
    echo $esp32c3_gcc_hash | sha256sum --check
    if [ $? -eq 0 ];then
        echo "Extracting riscv32-esp-elf-gcc"
        tar -xf ${esp32c3_gcc_filename} -C ~/.local
        ~/.local/riscv32-esp-elf/bin/riscv32-esp-elf-gcc -v &> /dev/null
        if [ $? -eq 0 ];then
            echo "Extracting ok"
            rm -f ${esp32c3_gcc_filename}
        fi
    fi
else
    echo "Compiler already installed"
fi

python3 -m pip install --upgrade pip

python3 -m pip install --no-warn-script-location -r ${script_dir}/requirements.txt --upgrade --constraint ${script_dir}/constraints.v5.0.txt --extra-index-url https://dl.espressif.com/pypi

echo "Installation completed"
echo "Please run 'source build/env.sh' to import build environment"