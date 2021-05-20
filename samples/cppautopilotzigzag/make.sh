#
# (c) https://github.com/MontiCore/monticore
#


SHARED_CPP_DIR=../../extern/shared_cpp

g++ -shared -fPIC -std=c++11 -static-libstdc++ -O3 -I${SHARED_CPP_DIR} -o "cppautopilot_lib.so" "library_interface.cpp" "autopilot.cpp" "${SHARED_CPP_DIR}/buffer.cpp" "${SHARED_CPP_DIR}/json.cpp" "${SHARED_CPP_DIR}/printf.cpp"
