cmake_minimum_required(VERSION 3.13)
project(fp)
set(CMAKE_CXX_STANDARD 17)
set(SOURCE_FILES main.cpp)
add_executable(fp ${SOURCE_FILES})

add_subdirectory(src)

######
# If you are on the Lab Machines, or have installed the OpenGL libraries somewhere
# other than on your path, leave the following two lines uncommented and update
# the path.  The current path is for the lab environments.
#
# If your installations are on your path, then comment these two lines
######

include_directories("C:/CSCI441/include")
target_link_directories(fp PUBLIC "C:/CSCI441/lib")

#include_directories("/usr/local/include")
#target_link_directories(lab1 PUBLIC "/usr/local/lib")

# the following line is linking instructions for Windows.  comment if on OS X, otherwise leave uncommented
target_link_libraries(fp stbimage opengl32 glfw3 glew32.dll gdi32 freetype)

# the following line is linking instructions for OS X.  uncomment if on OS X, otherwise leave commented
#target_link_libraries(lab1 "-framework OpenGL" glfw3 "-framework Cocoa" "-framework IOKit" "-framework CoreVideo" glew stbimage freetype bz2 z)
