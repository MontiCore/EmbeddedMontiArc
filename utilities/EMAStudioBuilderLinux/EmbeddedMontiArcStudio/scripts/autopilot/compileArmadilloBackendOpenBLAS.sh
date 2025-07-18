rm -rf "${DLL_DIR}"
# (c) https://github.com/MontiCore/monticore  
mkdir "${DLL_DIR}"

g++ -std=c++11 -shared -fPIC \
   -I"${JAVA_HOME}/include" \
   -I"${JAVA_HOME}/include/linux" \
   -I"${ARMADILLO_HOME}/include" \
   -L"${HOME}/lib-linux" \
   -o "${DLL_DIR}/AutopilotAdapter.dll" \
   "${CPP_DIR}/AutopilotAdapter.cpp" \
   -DARMA_DONT_USE_WRAPPER -lopenblas -larpack
