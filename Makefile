all: build connect run

# C++ build, run for real time control
build:
	make -C ./build

start:
	./start_can.sh

run:
	./build/source/quadson

stop:
	./stop_can.sh

clean:
	make -C ./build clean

# Python build, run for simulation	
Install:
	python3 -m venv venv # Create
	pip install -r requirements.txt # Install

sim:
	source venv/bin/activate && python3 ./python_scripts/sim.py
