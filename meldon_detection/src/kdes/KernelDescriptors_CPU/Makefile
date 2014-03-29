.PHONY: all
.DEFAULT: all

CXXFLAGS =  -msse2 -msse3 -msse4 # -march=native
CXXFLAGS += -fpermissive -O3 -IInclude -lmatio -lboost_thread -lboost_filesystem -lboost_system `pkg-config opencv matio --cflags --libs`

all: lib kernelmain computekdes
lib: libkerneldesc.a
libkerneldesc.a: libkerneldesc.cc
	g++ -c libkerneldesc.cc $(CXXFLAGS)
	ar rvs libkerneldesc.a libkerneldesc.o
#	g++ -g -c libkerneldesc.cc -march=native -IInclude -lmatio `pkg-config opencv matio --cflags --libs`
kernelmain: kernelmain.cc libkerneldesc.a
	g++ -o kernelmain kernelmain.cc libkerneldesc.a $(CXXFLAGS)
#	g++ -g -o kernelmain kernelmain.cc libkerneldesc.a -IInclude -lmatio -lboost_thread -lboost_filesystem -lboost_system `pkg-config opencv matio --cflags --libs`
#	g++ -O2 -o kernelmain -lmatio -lhdf5 -lcv -lhighgui -IInclude kernelmain.cc
#	g++ -O3 -o kernelmain kernelmain.cc -IInclude -lmatio -fopenmp `pkg-config opencv matio --cflags --libs`
#	g++ -O3 -march=native -o kernelmain kernelmain.cc -IInclude -lmatio -fopenmp `pkg-config opencv matio --cflags --libs`
#	g++ -g -o kernelmain kernelmain.cc libkerneldesc.cc -IInclude -lmatio `pkg-config opencv matio --cflags --libs`
#	g++ -O3 -march=native -o kernelmain kernelmain.cc libkerneldesc.cc -IInclude -lmatio `pkg-config opencv matio --cflags --libs`
computekdes: computekdes.cc libkerneldesc.a
	g++ -o computekdes computekdes.cc libkerneldesc.a $(CXXFLAGS)
#mattest: mattest.cc
#	g++ -O2 -o mattest mattest.cc `pkg-config matio --cflags --libs` -lhdf5 -IInclude 
#matop: matop.cc
#	g++ -O3 -o matop -fopenmp -IInclude matop.cc
clean:
	rm -f kernelmain computekdes libkerneldesc.o libkerneldesc.a
