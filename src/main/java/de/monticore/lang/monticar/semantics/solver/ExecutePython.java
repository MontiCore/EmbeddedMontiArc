/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.PumpStreamHandler;
import org.apache.commons.io.output.ByteArrayOutputStream;

import java.io.IOException;

public class ExecutePython {

    public static String executePython(String file) {
        String line = "python " + resolvePythonScriptPath(file);
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
        return "src/test/resources/de/monticore/lang/monticar/semantics/python/" + relativePath;
    }

}
