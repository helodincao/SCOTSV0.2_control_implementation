#
# matlab root 
#
#Mac
#MATLABROOT	= /Applications/MATLAB_R2021a.app

#linux
MATLABROOT	= "/mnt/c/Program Files/MATLAB/R2019b"

#
# compiler
#
CC       	  = $(MATLABROOT)/bin/mex
CXXFLAGS 		= -g -Wall -Wextra -std=c++14 -O3 
#
# scots
#
SCOTSROOT   = ../..
SCOTSINC		= -I$(SCOTSROOT)/src2
#
# cudd 
#
# CUDDPATH		=  /opt/local/
# CUDDINC 		= -I$(CUDDPATH)/include
# CUDDLIBS		= -lcudd 
# CUDDLPATH   = -L$(CUDDPATH)/lib



# all: sparse bdd
all: sparse

sparse: GridPoints mexStaticController

# bdd: mexSymbolicSet


mexStaticController: mexStaticController.cc
	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(SCOTSINC) mexStaticController.cc

# Commented on August 20, 2021. To make it work with matlabR2021a
# GridPoints: GridPoints.cc
# 	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(SCOTSINC) GridPoints.cc

GridPoints: GridPoints.cc
	$(CC) CXXFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -largeArrayDims $(SCOTSINC) GridPoints.cc

# mexSymbolicSet: mexSymbolicSet.cc
# 	$(CC) COMPFLAGS='$(COMPFLAGS) $(CXXFLAGS)' -DSCOTS_BDD -largeArrayDims $(SCOTSINC) $(CUDDINC) $(CUDDLPATH) $(CUDDLIBS) mexSymbolicSet.cc

