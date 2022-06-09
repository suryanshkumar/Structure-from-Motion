# Adds a library called Hello (libHello.a under Linux) from the source file hello.cc
include_directories(
    /usr/local/include
    /usr/include/gdal)
ADD_LIBRARY( Sparse3D

        #L1 Tracker
        basicSfM
        essential
        triangulate
        #BlobLib

        #configRead
	      )

