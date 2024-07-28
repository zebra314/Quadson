all: build start

# C++ build, run for real time control
build:
	docker buildx build --platform linux/arm/v7 --load -t quadson:latest .

start:
	docker run --rm -it \
			--platform linux/arm/v7 \
			--name quadson \
			--network=host \
			--cap-add=NET_ADMIN \
			--device /dev/:/dev/ \
			-v /tmp/:/tmp/ \
			-v /run/udev/:/run/udev/ \
			--mount type=bind,source=$(CURDIR)/quadson,target=/quadson \
			quadson:latest \
    	/bin/bash -c "clear; exec bash"

run:
	# ./build/source/quadson

stop:
	# ./stop_can.sh

clean:		
	# make -C ./build clean

# Python build, run for simulation	
install-python:
	# python3 -m venv venv # Create
	# pip install -r requirements.txt # Install

sim:
	# source venv/bin/activate && python3 ./python_scripts/sim.py
