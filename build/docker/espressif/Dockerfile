FROM ubuntu:18.04

LABEL maintainer="baidongyang@jeejio.com"

# src 参数必需与 docker run -v 冒号右侧的路径相同
# docker build -t ${USER}/swl:1.0 . --build-arg user=${USER} --build-arg password=<PASSWORD> --build-arg uid=${UID} --build-arg gid=$(id -g) --build-arg home=${HOME} --build-arg
# docker run -it --name="<CONTAINER NAME>" -v ${HOME}/jenkins-script/SlenderWestLakeOS/src/slenderwestlake_edu_lan_v1.0_intra:${HOME}/slenderwestlake_edu_lan_v1.0_intra ${USER}/swl:1.0 /bin/bash
# docker start -i <CONTAINER ID>

ARG user=user
ARG password=user
ARG uid=1000
ARG gid=1000
ARG home="/home/user"

# Set default timezone, Quietly install tzdata.
ENV DEBIAN_FRONTEND noninteractive

# Replace software source - http.
RUN mv /etc/apt/sources.list /etc/apt/sources.list.bak
COPY sources.list.http.18.04 /etc/apt/sources.list
RUN apt-get update

# Install dependent packages.
RUN apt-get install -y apt-utils
RUN apt-get install -y sudo vim curl net-tools iputils-ping ssh bash-completion tzdata tree mlocate wget gperf pigz
RUN apt-get install -y git make autoconf autogen libtool libsubunit-dev gawk cpio bison flex lib32stdc++6 xz-utils build-essential
RUN apt-get install -y ninja-build ccache libffi-dev dfu-util libusb-1.0-0
# esp-idf
# 必需先安装 python3.7, 再安装 python3-venv, 因为 python3-venv 默认安装的是 python3.6
RUN apt-get install -y python3.7 python3-venv python3-pip
RUN ln -sf /usr/bin/python3.7 /usr/bin/python3

# Install cmake
WORKDIR /root
RUN apt-get install -y libssl-dev
COPY cmake-3.25.2.tar.gz /root
RUN tar xzvf cmake-3.25.2.tar.gz
WORKDIR /root/cmake-3.25.2
RUN ./bootstrap
RUN make -j8
RUN make install
RUN ln -sf /usr/local/bin/cmake /usr/bin/cmake
WORKDIR /
RUN rm -rfv /root/cmake-3.25.2 /root/cmake-3.25.2.tar.gz

RUN ln -sf /usr/lib/x86_64-linux-gnu/libmpfr.so.6 /usr/lib/libmpfr.so.4

# Modify timezone.
RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime
# Make timezone settings effective.
RUN dpkg-reconfigure -f noninteractive tzdata

# Replace software source - https.
RUN apt-get install -y apt-transport-https
COPY sources.list.https.18.04 /etc/apt/sources.list
RUN apt-get update

# Add user, Maintain consistency with the host.
RUN mkdir -p ${home}
RUN useradd --home ${home} --shell /bin/bash -p ${password} ${user}
RUN echo "${user}:${password}" | chpasswd

# Modify UID and GID to be consistent with the host.
RUN usermod -u ${uid} ${user}
RUN groupmod -g ${gid} ${user}

RUN chown -R ${user}:${user} ${home}

RUN adduser ${user} sudo

# Modify sh link from dash to bash.
# looklike "dpkg-reconfigure dash" change to "no".
RUN ln -sf /bin/bash /bin/sh
RUN ln -sf /bin/dash /bin/sh.distrib
RUN ln -sf /usr/share/man/man1/dash.1.gz /usr/share/man/man1/sh.distrib.1.gz
RUN ln -sf /usr/share/man/man1/bash.1.gz /usr/share/man/man1/sh.1.gz

RUN updatedb

USER ${user}
WORKDIR ${home}

COPY bashrc .bashrc

COPY local.tar.gz .
RUN pigz -d local.tar.gz
RUN tar xvf local.tar
RUN rm -vf local.tar

WORKDIR ${home}
CMD /bin/bash
