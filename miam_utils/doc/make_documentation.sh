#!/bin/bash
BBB_PROJECT_NUMBER="$(grep "project([a-zA-Z]* VERSION " ../CMakeLists.txt)"
BBB_PROJECT_NUMBER=${BBB_PROJECT_NUMBER%)*}  # retain the part before the closed parenthesis 
BBB_PROJECT_NUMBER=${BBB_PROJECT_NUMBER##*VERSION }  # retain the part after VERSION string
export BBB_PROJECT_NUMBER = $BBB_PROJECT_NUMBER
exec doxygen
