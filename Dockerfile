FROM navikey/raspbian-buster:latest

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
    make \
    build-essential \
    cmake \
    can-utils \
    libeigen3-dev

WORKDIR /quadson
