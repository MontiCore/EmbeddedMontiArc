/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.execution;

import jline.internal.Log;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Optional;
import java.util.Scanner;

public class ExecutePython extends ExecuteAbs {

    public static String executePython(String scriptFile, String command) {
        if (isPythonInstalled()) {
            String line = "python " + resolvePythonScriptPath(scriptFile) + " " + command;
            return executeCommand(line, true);
        } else {
            Log.warn("python is not installed.");
            return "-1";
        }
    }

    private static String standardPath = "de/monticore/lang/monticar/semantics/scripts/python";

    private static String resolvePythonScriptPath(String relativePath) {
        if (relativePath.endsWith(".py"))
            relativePath = relativePath.substring(0, relativePath.lastIndexOf("."));
        String res = relativePath.replace(".", "/").replace("\\", "/");
        res = res + ".py";

        Optional<File> file = searchIn(res, standardPath, "src/main/resources/" + standardPath, "target/classes/" + standardPath);

        if (!file.isPresent() || !file.get().exists()) {
            Log.error("Cannot find python script: " + relativePath);
            return "";
        }
        return file.get().getAbsolutePath();
    }

    private static boolean isPythonInstalled() {
        String commandString;
        if (System.getProperty("os.name").toLowerCase().contains("windows")) {
            commandString = "cmd.exe /C python --version";
        } else {
            commandString = "python --version";
        }

        if (executeCommand(commandString, true).startsWith("Python 3."))
            return true;
        else {
            Log.error("could not retrieve python version");
            return false;
        }
    }

}
