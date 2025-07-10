/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class LibraryService {
    public static enum LibraryExceptionType {
        OS_RESOLVE, ACCESS_RESOURCE, WRITE_LIBRARY
    }

    static class SystemInfo {
        public final String workingDirectory;
        public final String systemName;
        public final String libraryExtension;

        public SystemInfo() {
            workingDirectory = System.getProperty("user.dir") + "/";
            String OS = System.getProperty("os.name").toLowerCase();
            if (OS.contains("win")) {
                systemName = "windows";
                libraryExtension = ".dll";
            } else if (OS.contains("nix") || OS.contains("nux") || OS.contains("aix")) {
                systemName = "linux";
                libraryExtension = ".so";
            } else if (OS.contains("mac")) {
                systemName = "mac";
                libraryExtension = ".dylib";
            } else
                throw new ExceptionInInitializerError(new LibraryException(LibraryExceptionType.OS_RESOLVE, OS));
        }
    }

    static final SystemInfo systemInfo = new SystemInfo();

    public static class LibraryException extends Exception {
        private static final long serialVersionUID = -602493909994696880L;
        LibraryExceptionType type;
        String details;

        public LibraryException(LibraryExceptionType type, String details) {
            this.type = type;
            this.details = details;
        }

        public String toString() {
            switch (type) {
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

    public static String getWorkingDirectory() {
        return systemInfo.workingDirectory;
    }

    public static String getSystemName() {
        return systemInfo.systemName;
    }

    public static String getLibraryExtension() {
        return systemInfo.libraryExtension;
    }

    public static String getSystemLibraryName(String lib_name) {
        return lib_name + systemInfo.libraryExtension;
    }

    /**
     * Exports a resource as file. If 'systemDependent' is true it will lookup
     * 'libName' under a "windows", "linux" or "mac" folder depending on the system.
     * (libName can contain a relative path plus the library name. The file will
     * have the same relative path to the working directory.)
     */
    public static void prepareLibrary(String libName, boolean systemDependent) throws LibraryException {
        String libPath = getWorkingDirectory() + libName;
        // System.out.println("lib_path: " + lib_path);
        File targetFile = new File(libPath);
        if (targetFile.exists() && !targetFile.isDirectory()) {
            return;
        }
        // Write library to disk
        try {
            String pathToResource = systemDependent ? "/" + getSystemName() + "/" + libName : "/" + libName;
            InputStream res = LibraryService.class.getResourceAsStream(pathToResource);
            targetFile.getParentFile().mkdirs();
            FileOutputStream fout = new FileOutputStream(targetFile);
            BufferedOutputStream out = new BufferedOutputStream(fout);
            int data = res.read();
            while (data != -1) {
                out.write(data);
                data = res.read();
            }
            res.close();
            out.close();
            fout.close();
        } catch (IOException e) {
            throw new LibraryException(LibraryExceptionType.WRITE_LIBRARY, libName + (" (" + e + ")"));
        }
        System.out.println("Exported library \"" + libName + "\" from the resources.");
    }

}
