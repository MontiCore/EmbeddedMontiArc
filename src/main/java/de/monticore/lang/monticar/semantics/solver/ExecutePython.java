/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver;

import jline.internal.Log;
import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.PumpStreamHandler;
import org.apache.commons.io.output.ByteArrayOutputStream;

import java.io.File;
import java.io.IOException;

public class ExecutePython {

    public static String executePython(String file, String command) {
        String line = "python " + resolvePythonScriptPath(file) + " " + command;
        CommandLine cmdLine = CommandLine.parse(line);

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        PumpStreamHandler streamHandler = new PumpStreamHandler(outputStream);

        DefaultExecutor executor = new DefaultExecutor();
        executor.setStreamHandler(streamHandler);

        int exitCode = 0;
        try {
            exitCode = executor.execute(cmdLine);
        } catch (IOException e) {
            e.printStackTrace();
        }

        if (exitCode == 0)
            return outputStream.toString();
        else
            return null;
    }

    private static String resolvePythonScriptPath(String relativePath) {
        String res = relativePath.replace(".", "/").replace("\\", "/");
        if (!res.endsWith(".py"))
            res = res + ".py";
        File file = new File(res);
        if (!file.exists())
            res = "src/main/resources/de/monticore/lang/monticar/semantics/scripts/python/" + res;
        file = new File(res);
        if (!file.exists())
            Log.error("Cannot find python file: " + relativePath);
        return res;
    }

}
