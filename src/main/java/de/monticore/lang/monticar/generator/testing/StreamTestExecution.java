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
package de.monticore.lang.monticar.generator.testing;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;
import org.apache.commons.lang3.SystemUtils;

import java.io.*;

/**
 * @author Sascha Schneiders
 */
public class StreamTestExecution {

    public static void compileTests(String targetFullPath, String targetBasePath) throws IOException {
        File f = new File( OSHelper.getDirPrefix() + "/"+targetBasePath + "/exec");
        if (!f.exists()) {
            f.mkdirs();
            f.mkdir();
        }
        String execName = "";
        if (SystemUtils.IS_OS_WINDOWS) {
            execName = "compileCPPTests.bat";
        } else if (SystemUtils.IS_OS_LINUX) {
            execName = "./compileCPPTests.sh";
        } else {
            execName = "generic_platform";
            System.out.println("ERROR: Unsupported platform!!!");
        }

        //Check if g++ is available
        //will throw IOException if not found
        Runtime.getRuntime().exec("g++ --version");

        Process p = Runtime.
                getRuntime().
                exec(new String[]{execName, targetFullPath, targetBasePath + "/exec"});
        execWithExitCheck(p, "Compiler error:");

    }

    private static void execWithExitCheck(Process p, String errorPrefix) throws IOException {

        while (p.isAlive()) {
            //if (Log.isInfoEnabled(""))
            {
                char read = (char) p.getInputStream().read();
                if(isSafeToPrint(read)) {
                    System.out.print(read);
                }
            }
        }

        while (p.getInputStream().available() > 0){
            char read = (char) p.getInputStream().read();
            if(isSafeToPrint(read)) {
                System.out.print(read);
            }
        }

        if(p.exitValue() != 0){
            StringBuilder errorStream = new StringBuilder();
            while(p.getErrorStream().available() > 0){
                char read = (char) p.getErrorStream().read();
                if(isSafeToPrint(read)) {
                    errorStream.append(read);
                }
            }
            Log.error(errorPrefix + "\n" + errorStream.toString());
        }
    }

    private static boolean isSafeToPrint(char c){
        String cs = "" + c;
        return StringUtils.isWhitespace(cs) || StringUtils.isAsciiPrintable(cs);
    }

    public static void executeTests(String targetBasePath) throws IOException {

        String execName = "";
        if (SystemUtils.IS_OS_WINDOWS) {
            execName = "executeStreamTest.bat";
        } else if (SystemUtils.IS_OS_LINUX) {
            execName = "./executeStreamTest.sh";
        } else {
            execName = "generic_platform";
            System.out.println("ERROR: Unsupported platform!!!");
        }
        Process p = Runtime.
                getRuntime().
                exec(new String[]{execName, targetBasePath + "/exec"});

        execWithExitCheck(p, "Execution error:");


    }
}
