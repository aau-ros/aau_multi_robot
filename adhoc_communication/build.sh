#!/bin/bash

clear

if [ -z $1 ]; then
	rm -rf build/
	rm -rf bin/
	rm -f CMakeCache.txt
	rm -rf srv_gen/
	rm -rf msg_gen/
	rm -rf test_results/
	rm -rf devel/
	rm -rf lib/
	rm -rf gtest/
	rm -rf catkin/
	rm -rf bin/
	rm -rf include/
	rm -rf CMakeFiles/
	rm -rf catkin_generated/
	rm -rf build/

	rm -rf Makefile
	rm -rf cmake_install.cmake
fi

cd ../..
catkin_make --force-cmake


echo "Root password is needed to set SUID bit of the binary file to use raw socket connections.."
sudo chown root ./devel/lib/adhoc_communication/adhoc_communication
sudo chmod +s ./devel/lib/adhoc_communication/adhoc_communication

cd - 
 
