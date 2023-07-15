CC=g++

help:
	@echo "To compile:             make compile"
	@echo "To compile and run:     make run"

compile: mock.cpp mock.hpp ../motor_control.cpp
	$(CC) -o out/mock mock.cpp

run: compile
	./out/mock > ./out/out.txt
	@echo "ERROR Count: "
	@grep -c "ERROR" ./out/out.txt || true
	@echo "Test done!"
