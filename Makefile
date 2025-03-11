all: sim

# ---------------------------------- Native ---------------------------------- #
# Launch on the raspberry pi

nav:
	$(MAKE) nav-connect
	$(MAKE) nav-run

nav-connect:
	./start_can.sh
nav-run:
	./build/source/quadson
nav-pause:
	./build/source/quadson -p
nav-disconnect:
	./stop_can.sh
nav-clean:		
	make -C ./build clean

# --------------------------------- Emulation -------------------------------- #
# C++ build, run for real time control on PC (x86) using docker

emu:
	$(MAKE) emu-build
	$(MAKE) emu-start

emu-build:
	docker buildx build -f Dockerfile.emu \
		--platform linux/arm/v7 \
		--load -t quadson-emu:latest .

emu-start:
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
	docker run --rm -it \
		--platform linux/arm/v7 \
		--name quadson-emu \
		--network=host \
		--cap-add=NET_ADMIN \
		--device /dev/:/dev/ \
		-v /tmp/:/tmp/ \
		-v /run/udev/:/run/udev/ \
		--mount type=bind,source=$(CURDIR)/quadson,target=/quadson \
		quadson-emu:latest

# -------------------------------- Simulation -------------------------------- #
# Python build, run for simulation
.PHONY: sim sim-build-venv sim-start-venv sim-build-docker sim-start-docker

SIM_METHOD ?= venv # docker, venv

sim:
	@echo "Starting simulation with method: $(SIM_METHOD)"
	@$(MAKE) --no-print-directory sim-build-$(SIM_METHOD)
	@$(MAKE) --no-print-directory sim-start-$(SIM_METHOD)

sim-build-venv:
	@test -d venv || ( \
		echo "Creating virtual environment..." && \
		python3 -m venv venv && \
		. venv/bin/activate && \
		pip install --upgrade pip && \
		pip install -r config/requirements.txt \
	)

sim-start-venv:
	@. venv/bin/activate
	@echo "Virtual environment activated."

sim-build-docker:
	docker build -f config/Dockerfile.sim -t quadson-sim:latest .

sim-start-docker:
	xhost +local:root
	docker run -it --rm \
		--name quadson-sim \
		--mount type=bind,source=$(CURDIR)/quadson_sim,target=/quadson_sim \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		quadson-sim:latest
	xhost -local:root

# ------------------------------------ ROS ----------------------------------- #
# 

ros:
	$(MAKE) ros-build
	$(MAKE) ros-run
	$(MAKE) ros-clean

ros-build:
	docker build -f Dockerfile.ros -t ros-noetic-zsh:latest .

ros-run:
	xhost +local:root
	-docker run -it \
		--privileged \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		--mount type=bind,source=$(shell pwd)/quadson_ros,target=/root/quadson_ws \
		--user $(id -u):$(id -g) \
		--name ros-noetic-zsh \
		ros-noetic-zsh:latest
	xhost -local:root

ros-clean:
	docker container rm ros-noetic-zsh
