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

sim:
	$(MAKE) sim-build
	$(MAKE) sim-start

sim-build:
	docker build -f Dockerfile.sim -t quadson-sim:latest .

sim-start:
	xhost +local:root
	docker run -it --rm \
		--name quadson-sim \
		--mount type=bind,source=$(CURDIR)/quadson-sim,target=/quadson-sim \
		--env="DISPLAY" \
		-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
		-e XDG_RUNTIME_DIR=/tmp \
		-e QT_X11_NO_MITSHM=1 \
		quadson-sim:latest
	xhost -local:root

