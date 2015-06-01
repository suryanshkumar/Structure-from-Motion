# StructurefromMotion

Author: Suryansh Kumar

This code implements basic SfM pipeline. I hope this code 
will be useful to students and reseachers who wants to understand 
or revise basic SfM. I have tried not to use template programming 
as few people find it difficult to undertand. 


To quickly check the code, few sample images and its intrinsic matrix 
is provided with this code. This code has implemented the theory described 
in "Multiple View Geometry" text book by Richard Hartley and Andrew Zisserman. 
I have also taken help from "A structure and Motion Toolkit in Matlab" 
developed by P.H.S Torr while implementing this code.


Dependencies:
a) cmake
b) OpenCV (C++) (Tested on 2.4.10 and 3.0 version)

To view 3D points
-> I have used MeshLab. It's screenshot is provided in output folder.

Compilation Instructions
a) cd StructurefromMotion
b) mkdir build
c) cd build
d) cmake ..
e) make

To execute binary
a) go to app folder inside build directory (cd apps)
b) ./reconstruction
To save your output to a file
c) ./reconstruction > output.xyz

Use MeshLab to view .xyz file.




