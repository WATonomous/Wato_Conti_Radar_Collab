#!/bin/bash

g++ -I include/ main.cpp src/dbscan.cpp src/cluster.cpp src/point.cpp src/evaluate.cpp -o dbscan.out

./dbscan.out
