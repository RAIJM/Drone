#!/bin/bash
g++ -c Matrix.cpp -o matrix.o
g++ -c main.cpp -o main.o
g++ matrix.o main.o -o main
./main
