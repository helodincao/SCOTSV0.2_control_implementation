CXX ?= g++

CXXFLAGS := -std=c++11 -O3 -DNDEBUG -I../../src -I../../utils

go2_controller: go2_controller.cc
	$(CXX) $(CXXFLAGS) go2_controller.cc -o go2_controller

clean:
	rm -f go2_controller