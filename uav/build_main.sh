#!/bin/bash
#g++ -c Matrix.cpp -o matrix.o
#g++ -c main.cpp -o main.o
#g++ -c KalmanFilter.cpp -o kalman.o
#g++ matrix.o kalman.o main.o -o main
g++ -c PID.cpp -o pid.o
g++ -c main.cpp -o main.o
g++ pid.o main.o -o main
./main
