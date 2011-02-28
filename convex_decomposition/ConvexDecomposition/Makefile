

OBJS = DecomposeSample.o \
	ConvexDecomposition/bestfit.o              ConvexDecomposition/float_math.o \
	ConvexDecomposition/bestfitobb.o           ConvexDecomposition/meshvolume.o \
	ConvexDecomposition/cd_hull.o              ConvexDecomposition/planetri.o \
	ConvexDecomposition/cd_wavefront.o         ConvexDecomposition/raytri.o \
	ConvexDecomposition/concavity.o            ConvexDecomposition/splitplane.o \
	ConvexDecomposition/ConvexDecomposition.o  ConvexDecomposition/triangulate.o \
	ConvexDecomposition/fitsphere.o            ConvexDecomposition/vlookup.o

HEADERS = \
	ConvexDecomposition/bestfit.h \
	ConvexDecomposition/bestfitobb.h \
	ConvexDecomposition/cd_hull.h \
	ConvexDecomposition/cd_vector.h \
	ConvexDecomposition/cd_wavefront.h \
	ConvexDecomposition/concavity.h \
	ConvexDecomposition/ConvexDecomposition.h \
	ConvexDecomposition/fitsphere.h \
	ConvexDecomposition/float_math.h \
	ConvexDecomposition/meshvolume.h \
	ConvexDecomposition/planetri.h \
	ConvexDecomposition/raytri.h \
	ConvexDecomposition/splitplane.h \
	ConvexDecomposition/triangulate.h \
	ConvexDecomposition/vlookup.h

CC = g++

DEBUG = -ggdb

CFLAGS = -IConvexDecomposition -Wall -c $(DEBUG)

LFLAGS = $(DEBUG)

convex_decomposition: $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o convex_decomposition

DecomposeSample.o: DecomposeSample.cpp
	$(CC) $(CFLAGS) DecomposeSample.cpp -o $@

ConvexDecomposition/bestfit.o: ConvexDecomposition/bestfit.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/bestfit.cpp -o $@

ConvexDecomposition/bestfitobb.o: ConvexDecomposition/bestfitobb.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/bestfitobb.cpp -o $@

ConvexDecomposition/cd_hull.o: ConvexDecomposition/cd_hull.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/cd_hull.cpp -o $@

ConvexDecomposition/cd_wavefront.o: ConvexDecomposition/cd_wavefront.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/cd_wavefront.cpp -o $@

ConvexDecomposition/concavity.o: ConvexDecomposition/concavity.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/concavity.cpp -o $@

ConvexDecomposition/ConvexDecomposition.o: ConvexDecomposition/ConvexDecomposition.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/ConvexDecomposition.cpp -o $@

ConvexDecomposition/fitsphere.o: ConvexDecomposition/fitsphere.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/fitsphere.cpp -o $@

ConvexDecomposition/float_math.o: ConvexDecomposition/float_math.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/float_math.cpp -o $@

ConvexDecomposition/meshvolume.o: ConvexDecomposition/meshvolume.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/meshvolume.cpp -o $@

ConvexDecomposition/planetri.o: ConvexDecomposition/planetri.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/planetri.cpp -o $@

ConvexDecomposition/raytri.o: ConvexDecomposition/raytri.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/raytri.cpp -o $@

ConvexDecomposition/splitplane.o: ConvexDecomposition/splitplane.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/splitplane.cpp -o $@

ConvexDecomposition/triangulate.o: ConvexDecomposition/triangulate.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/triangulate.cpp -o $@

ConvexDecomposition/vlookup.o: ConvexDecomposition/vlookup.cpp ConvexDecomposition/vlookup.cpp
	$(CC) $(CFLAGS) ConvexDecomposition/vlookup.cpp -o $@

install:
	cp convex_decomposition ../../convex_decomposition/bin/

clean:
	\rm *.o */*.o convex_decomposition

tar:
	tar cfv ConvexDecomposition.tar DecomposeSample.cpp convex_decomposition Makefile \
		ConvexDecomposition
