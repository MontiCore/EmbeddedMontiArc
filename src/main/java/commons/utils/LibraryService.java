/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package commons.utils;

import java.io.*;
import java.nio.channels.FileChannel;
import java.net.URL;

public class LibraryService {
    public static enum LibraryExceptionType {
        OS_RESOLVE,
        ACCESS_RESOURCE,
        WRITE_LIBRARY
    }
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
        return System.getProperty("user.dir") + "\\";
    }

    public static String getSystemLibraryName(String lib_name) throws LibraryException {
        String OS = System.getProperty("os.name").toLowerCase();
        if (OS.contains("win")){
            return lib_name+".dll";
        } else if (OS.contains("nix")|| OS.contains("nux")|| OS.contains("aix")){
            return lib_name+".so";
        }
        throw new LibraryException(LibraryExceptionType.OS_RESOLVE, OS);
    }

    public static void prepareLibrary(String working_dir, String lib_name) throws LibraryException {
        String lib_path = working_dir+lib_name;
        //System.out.println("lib_path: " + lib_path);
        File target_file = new File(lib_path);
        if(target_file.exists() && !target_file.isDirectory()) {
            return;
        }
        //Write library to disk
        try {
            InputStream res = LibraryService.class.getResourceAsStream("/"+lib_name);
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
