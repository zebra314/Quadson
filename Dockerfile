FROM navikey/raspbian-buster:latest

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    make \
    build-essential \
    cmake \
    can-utils \
    git

RUN git clone -b 3.3.9 https://gitlab.com/libeigen/eigen.git
RUN cp -r eigen/Eigen /usr/include/

WORKDIR /quadson