This folder contain the marker 1 test and source files of the vision part of the project.


To compile on linux:

cmkdir build
cd build
cmake ..
make



The scrips runMarkerTest and runMarkerTestHard generate test images in the result folder for easy and hard sequences. They also generate a runtime csv file. It should be noted that the scripts uses absolute paths, so if the provided test image sequences are not in /home/student/Desktop/workspace/marker_color/ and /home/student/Desktop/workspace/marker_color/ then these paths should be changed accrodingly.


