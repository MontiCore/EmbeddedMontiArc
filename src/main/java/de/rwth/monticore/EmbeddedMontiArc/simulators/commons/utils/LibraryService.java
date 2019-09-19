/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils;

import java.io.*;
import java.nio.channels.FileChannel;
import java.net.URL;

public class LibraryService {
    public static enum LibraryExceptionType {
        OS_RESOLVE,
        ACCESS_RESOURCE,
        WRITE_LIBRARY
    }

    public static class SystemInfo{
        public final String workingDirectory;
        public final String systemName;
        public final String libraryExtension;
        public SystemInfo() {
            workingDirectory = System.getProperty("user.dir") + "/";
            String OS = System.getProperty("os.name").toLowerCase();
            if (OS.contains("win")){
                systemName = "windows";
                libraryExtension = ".dll";
            } else if (OS.contains("nix") || OS.contains("nux") || OS.contains("aix") ){
                systemName = "linux";
                libraryExtension = ".so";
            } else if (OS.contains("mac")){
                systemName = "mac";
                libraryExtension = ".dylib";
            }
            else throw new ExceptionInInitializerError(new LibraryException(LibraryExceptionType.OS_RESOLVE, OS));
        }
    }

    public static final SystemInfo systemInfo = new SystemInfo();

    public static class LibraryException extends Exception {
        LibraryExceptionType type;
        String details;
        public LibraryException(LibraryExceptionType type, String details){
            this.type = type;
            this.details = details;
        }
        public String toString(){
            switch (type){
                case OS_RESOLVE:
                    return "Could not deduce Operating System library extension from: " + details + ".";
                case ACCESS_RESOURCE:
                    return "Could not get resource: " + details + ".";
                case WRITE_LIBRARY:
                    return "Could not write Library to disk: " + details + ".";
            }
            return "Unknown LibraryException.";
        }
    }

    public static String getWorkingDirectory(){
        return systemInfo.workingDirectory;
    }

    public static String getSystemName(){
        return systemInfo.systemName;
    }

    public static String getLibraryExtension(){
        return systemInfo.libraryExtension;
    }

    public static String getSystemLibraryName(String lib_name) {
        return lib_name + systemInfo.libraryExtension;
    }

    /*
        Exports a resource as file.
        If system_dependent is true it will lookup "lib_name" under "windows", "linux" or "mac" depending on the system.
        (lib_name can contain a relative path plus the library name. The file will have the same relative path to the working directory.)
    */
    public static void prepareLibrary(String lib_name, boolean system_dependent) throws LibraryException {
        String lib_path = getWorkingDirectory()+lib_name;
        //System.out.println("lib_path: " + lib_path);
        File target_file = new File(lib_path);
        if(target_file.exists() && !target_file.isDirectory()) {
            return;
        }
        //Write library to disk
        try {
            String path_to_resource = system_dependent ? "/"+getSystemName()+"/"+lib_name : "/"+lib_name;
            InputStream res = LibraryService.class.getResourceAsStream(path_to_resource);
            target_file.getParentFile().mkdirs();
            FileOutputStream fout= new FileOutputStream(target_file);
            BufferedOutputStream out = new BufferedOutputStream(fout);
            int data = res.read();
            while(data != -1) {
                out.write(data);
                data = res.read();
            }
            res.close();
            out.close();
            fout.close();
        } catch (IOException e) {
            throw new LibraryException(LibraryExceptionType.WRITE_LIBRARY, lib_name + (" (" + e + ")"));
        }
        System.out.println("Exported library \"" + lib_name + "\" from the resources.");
    }

}
