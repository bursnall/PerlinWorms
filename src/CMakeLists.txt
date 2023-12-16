cmake_minimum_required(VERSION 3.13)
project(stbimage VERSION 1.0.0 DESCRIPTION "STB Image Library")

######
# If you are on the Lab Machines, or have installed the OpenGL libraries somewhere
# other than on your path, leave the following two lines uncommented and update
# the path.  The current path is for the lab environments.
#
# If your installations are on your path, then comment these two lines
######
include_directories("C:/CSCI441/include")
link_directories(stbimage PUBLIC "C:/CSCI441/lib")

#include_directories("/usr/local/include")
#link_directories(stbimage PUBLIC "/usr/local/lib")

add_library(stbimage STATIC stbimage.cpp)