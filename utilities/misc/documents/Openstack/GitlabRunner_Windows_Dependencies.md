<!-- (c) https://github.com/MontiCore/monticore -->
# Installation steps for gitlab runner for windows
Also see [Dependencies](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/INSTALL_DEPENDENCIES.md#windows)

## Dependencies
- Download and install/extract
    - [Gitlab Multirunner](https://rwth-aachen.sciebo.de/s/xWQ9LKMulFSi7x1/download?path=%2F&files=gitlabRunner.zip)
    - [Git](https://git-scm.com/download/win)
    - [Java JDK](https://rwth-aachen.sciebo.de/s/xWQ9LKMulFSi7x1/download?path=%2F&files=java.zip)
    - [Maven](https://maven.apache.org/download.cgi)
    - [Mingw64](https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fwin64&files=mingw64.zip)
    - [CMake](https://cmake.org/download/)
    - [Make](http://gnuwin32.sourceforge.net/packages/make.htm)
    - [Z3](https://github.com/Z3Prover/z3/releases)
    - [Armadillo](https://rwth-aachen.sciebo.de/s/igDWzLpdO5zYHBj/download?path=%2Fwin64&files=armadillo-8.200.2.zip)
    - [Visual Studio](https://visualstudio.microsoft.com/de/downloads)
        - At least MSVC v142 & Windows SDK are required
- Add to Path (Adapt for different installation paths)
    - C:\Program Files\Git\cmd
    - C:\Program Files\Java\jdk-version\bin
    - C:\Program Files\apache-maven-3.6.2\bin
    - C:\Program Files\mingw64\bin
    - C:\Program Files\CMake\bin
    - C:\Program Files (x86)\GnuWin32\bin
    - C:\Program Files\z3-version\bin
    - C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools
- Add environment variables
    - Armadillo_HOME = "armadillo installation base"
    - JAVA_HOME = C:\Program Files\Java\jdk-version
- Test with ([see also](https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/generators/EMAM2Middleware/blob/master/INSTALL_DEPENDENCIES.md#windows))
    - Java + Maven: `mvn -v`
    - `where g++`  
      C:\mingw64\bin\g++.exe  
      `where cmake`  
      C:\Program Files\CMake\bin\cmake.exe  
      `where make`  
      C:\Program Files\make-3.81-bin\bin\make.exe  
      `dir /b "%Armadillo_HOME%\include"`  
      armadillo  
      armadillo.h  
      ... 
    - ... 
- Config:
    - Powershell Admin:  
        `git config --system core.longpaths true`  
        `git config --global http.sslVerify false`
    - Windows long paths:
        - Hit the Windows key, type regedit.msc and press Enter.
        - Navigate to HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Control\FileSystem
        - Double click the LongPathsEnabled entry and set it to 1.
- Retrieve token for the runner and enter it in the `config.toml`
- Restart
- Run `start.bat`
