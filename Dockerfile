FROM navikey/raspbian-buster:latest

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update          
RUN apt-get install -y \
    build-essential \
    can-utils \
    git \
    cmake

# Install Eigen
RUN git clone -b 3.3.9 https://gitlab.com/libeigen/eigen.git
RUN cp -r eigen/Eigen /usr/include/

WORKDIR /quadson