#!/bin/bash

clean() {
	if [ -d "${JEEJIO_PATH}/out" ]
	then
		rm -rf "${JEEJIO_PATH}/out"
	fi
}

__build_init() {
	if [ ! -d "${target_dir}" ]
	then
		mkdir -p ${target_dir}
	fi
	if [ ! -f "${target_dir}/build.ninja" ]
	then
		cd ${target_dir}
		cmake -G Ninja -DESP_PLATFORM=1 -DJEEJIO_TARGET=esp32c3 -DCCACHE_ENABLE=1 ${JEEJIO_PATH}
		cd ${JEEJIO_PATH}
	fi
}

build() {
	__build_init
	ninja -C ${target_dir}
}

config() {
	ninja -C ${target_dir} menuconfig
}

flash() {
	ninja -C ${target_dir} flash
}

monitor() {
	ninja -C ${target_dir} monitor
}

__env_init() {
	export JEEJIO_PATH=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && cd ..; pwd )
	jeejio_compiler_esp32c3="riscv32-esp-elf"
	if [[ ! ${PATH} =~ ${jeejio_compiler_esp32c3} ]]
	then
		export PATH=${HOME}/.local/riscv32-esp-elf/bin:$PATH
	fi
	export target_dir=${JEEJIO_PATH}/out/target/esp32c3
	#export JEEJIOPORT="/dev/ttyUSB0"
	export JEEJIOBAUD=460800
	_env_init=1
}

if [ "${_env_init}" != "1" ]
then
	__env_init
	__build_init
	echo "-- Import environment complete"
	echo "Please run:"
	echo -e "    \033[1mconfig\033[0m  configure menuconfig"
	echo -e "    \033[1mbuild\033[0m   compile"
	echo -e "    \033[1mclean\033[0m   clear compiled file"
	echo -e "    \033[1mflash\033[0m   download image"
	echo -e "    \033[1mmonitor\033[0m monitor tool"
fi
