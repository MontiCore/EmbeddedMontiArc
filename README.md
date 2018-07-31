This maven plugin performs stream tests on EMAM components.



## Usage:

### pom.xml
Add this to your pom.xml
```
<!-- EMAM Streamtest -->
<build>
    <plugins>
    ...
    <plugin>
        <groupId>de.monticore.lang.montiarc.utilities</groupId>
        <artifactId>maven-streamtest</artifactId>
        <version>0.0.1-SNAPSHOT</version>
        <configuration>
            <pathMain>./src/main/emam</pathMain>
            <pathTest>./src/test/emam</pathTest>
            <pathTmpOut>./target/tmp</pathTmpOut>
        </configuration>
    
        <executions>
            <execution>
                <phase>test</phase>
                <goals>
                    <goal>emam-streamtest</goal>
                </goals>
            </execution>
        </executions>
    </plugin>
    ...
    </plugins>
</build>
```

## Environment Setup

### 1. CMake
Get and install cmake.

Link for all common operating system:
https://cmake.org/download/#latest

(optional)Add CMake to PATH environment variable.

### 2. Make & C++ Compilers 
#### 2.1 Windows
For best practise we use MinGW. 
A compilation/testing with VisualStudio is also possible but not recommended.
Here are the setup steps for MinGW:

1. Download MinGW 
    - http://www.mingw.org/
    - https://sourceforge.net/projects/mingw/files/latest/download?source=files

1. Install MinGW with the following settings. Otherwise a compilation of streamtest is not possible. 
    - Version: *NEWEST*
    - Architecture: x86_64
    - Threads: posix
    - Excpetion: seh
    - Build revision: 0
    
1. Add <PathToMinGWFolder>/bin to your PATH environment variable
    
    
#### 2.2 Linux & MacOs 
//TODO 


### 3. Armadillo
#### 3.1 Windows
For Windows we use MinGW. Here also VisualStudio building is possible but not recommended.
 
1. Download and unzip armadillo to a folder 
    - http://arma.sourceforge.net/download.html
1. With CMD go to armadillo folder and run (or just create a .bat file and place it into the armadillo folder):
    ```
    if EXIST build\ RMDIR /S /Q build\
    if EXIST bin\ RMDIR /S /Q bin\
    if EXIST lib\ RMDIR /S /Q lib\
    
    mkdir build
    cd build
    cmake .. -G "MinGW Makefiles"
    cmake --build . --target armadillo
    mkdir ..\bin
    mkdir ..\lib
    xcopy /s /y libarmadillo.dll ..\bin\*
    xcopy /s /y libarmadillo.dll ..\lib\*
    xcopy /s /y libarmadillo.dll.a ..\lib\*
    
    cd ..
    if EXIST build\ RMDIR /S /Q build\
    
    xcopy /s /y examples\lib_win64\* lib\*
    xcopy /s /y include\armadillo include\*.h
    ```
1. Create a new environment variable **Armadillo_HOME** and set it to <Path-To-Your-Armadillo-Folder> 
1. Restart your pc. (Java doesn't recognize changes in the enviroment variables)
#### 3.2 Linux

##### Pre-Built
For some operating linux based system there are pre-built packages:
- Fedora: https://apps.fedoraproject.org/packages/armadillo
- Debian: https://packages.debian.org/search?searchon=sourcenames&keywords=armadillo
- Ubuntu: https://launchpad.net/ubuntu/+source/armadillo/
- openSUSE: https://software.opensuse.org/package/armadillo
- Arch: https://aur.archlinux.org/packages/armadillo/

Make sure that there is an armadillo.h in the armadillo include folder.
If it is installed under /usr/local, there must be /usr/local/include/armadillo.h
If armadillo.h is not there:
- Create a link:
    ```
    ln -s /usr/local/include/armadillo.h /usr/local/include/armadillo 
    ``` 
Also if it is installed under /usr/local (or linked there) and /usr/local is in your 
PATH environment variable you don't need to create Armadillo_HOME. 

#####Manually
// TODO


#### 3.3 MacOS
Armadillo can be pre-built installed via Homebrew or MacPorts;
- Homebrew: https://formulae.brew.sh/formula/armadillo
- MacPorts: https://www.macports.org/ports.php?by=name&substr=armadillo

Or you can install it by hand if you use the steps for linux armadillo installation.  

### Problems 
#### CMake
If your cmake fails with an error message that your paths are to long you can edit the registry to overcome this problem.
(This only works under Windows 10.)

Link: https://www.howtogeek.com/266621/how-to-make-windows-10-accept-file-paths-over-260-characters/

