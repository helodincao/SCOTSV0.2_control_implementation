CXX ?= g++
CXXFLAGS := -std=c++11 -O3 -DNDEBUG -I../../src -I../../utils

simulate: simulate.cc
	$(CXX) $(CXXFLAGS) simulate.cc -o simulate

clean:
	rm -f simulate