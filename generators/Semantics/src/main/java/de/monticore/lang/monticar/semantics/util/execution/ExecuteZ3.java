/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.execution;

import jline.internal.Log;

import java.io.File;

public class ExecuteZ3 extends ExecuteAbs {

    public static final String DEFAULT_Z3_LOCATION = "target/generated-z3";

    public static String executeZ3(String scriptFile) {
        if (isZ3Installed()) {
            String line = "z3 " + resolveScriptPath(scriptFile);
            return executeCommand(line);
        } else {
            Log.warn("z3 is not installed.");
            return  "-1";
        }
    }

    private static String resolveScriptPath(String relativePath) {
        if (relativePath.endsWith(".z3"))
            relativePath = relativePath.substring(0, relativePath.lastIndexOf("."));
        String res = relativePath.replace(".", "/").replace("\\", "/");
        res = res + ".z3";
        File file = new File(res);
        if (!file.exists())
            res = DEFAULT_Z3_LOCATION + "/" + res;
        file = new File(res);
        if (!file.exists())
            Log.error("Cannot find z3 script: " + relativePath);
        return res;
    }

    /*
     * returns true, if z3 is installed. Therefore, it calls 'z3 -version' that should correctly output e.g. 'Z3 version 4.8.5 - 64 bit'
     */
    private static boolean isZ3Installed() {
        String commandString;
        if (System.getProperty("os.name").toLowerCase().contains("windows")) {
            commandString = "cmd.exe /C z3 -version";
        } else {
            commandString = "z3 -version";
        }

        if (executeCommand(commandString).toLowerCase().startsWith("z3 version"))
            return true;
        else {
            Log.error("could not retrieve python version");
            return false;
        }
    }
}
