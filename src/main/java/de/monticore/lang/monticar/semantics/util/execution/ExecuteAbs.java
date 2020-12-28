/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.execution;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.PumpStreamHandler;
import org.apache.commons.io.output.ByteArrayOutputStream;

import java.io.IOException;

public abstract class ExecuteAbs {

    protected static String executeCommand(String command, boolean handleError) {
        CommandLine cmdLine = CommandLine.parse(command);

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        PumpStreamHandler streamHandler = new PumpStreamHandler(outputStream);

        DefaultExecutor executor = new DefaultExecutor();
        executor.setStreamHandler(streamHandler);

        int exitCode = -1;
        try {
            exitCode = executor.execute(cmdLine);
        } catch (IOException e) {
//            Log.info("TODO Error while executing");
        }

        if (exitCode == 0)
            return outputStream.toString();
        else if (!handleError)
            return outputStream.toString();
        else
            return "-1";
    }
}
