/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.execution;

import jline.internal.Log;

import java.io.File;

public class ExecutePython extends ExecuteAbs {

    public static String executePython(String scriptFile, String command) {
        if (isPythonInstalled()) {
            String line = "python " + resolvePythonScriptPath(scriptFile) + " " + command;
            return executeCommand(line);
        } else {
            Log.warn("python is not installed.");
            return  "-1";
        }
    }

    private static String resolvePythonScriptPath(String relativePath) {
        if (relativePath.endsWith(".py"))
            relativePath = relativePath.substring(0, relativePath.lastIndexOf("."));
        String res = relativePath.replace(".", "/").replace("\\", "/");
        res = res + ".py";
        File file = new File(res);
        if (!file.exists())
            res = "src/main/resources/de/monticore/lang/monticar/semantics/scripts/python/" + res;
        file = new File(res);
        if (!file.exists())
            Log.error("Cannot find python script: " + relativePath);
        return res;
    }

    private static boolean isPythonInstalled() {
        String commandString;
        if (System.getProperty("os.name").toLowerCase().contains("windows")) {
            commandString = "cmd.exe /C python --version";
        } else {
            commandString = "python --version";
        }

        if (executeCommand(commandString).startsWith("Python 3."))
            return true;
        else {
            Log.error("could not retrieve python version");
            return false;
        }
    }

}
