/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.run.python;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.ExecuteException;
import org.apache.commons.exec.PumpStreamHandler;
import org.apache.commons.io.output.ByteArrayOutputStream;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.assertEquals;

public class RunPythonTest {

    @Test
    public void test() throws ExecuteException, IOException {
        String line = "python " + resolvePythonScriptPath("dglSolver_01.py");
        CommandLine cmdLine = CommandLine.parse(line);

        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        PumpStreamHandler streamHandler = new PumpStreamHandler(outputStream);

        DefaultExecutor executor = new DefaultExecutor();
        executor.setStreamHandler(streamHandler);

        int exitCode = executor.execute(cmdLine);
        assertEquals("No errors should be detected", 0, exitCode);
        assertEquals("Should contain script output: ", "Hello Baeldung Readers!!", outputStream.toString()
                .trim());
    }

    private String resolvePythonScriptPath(String relativePath) {
        return "src/test/resources/de/monticore/lang/monticar/semantics/python/" + relativePath;
    }
}
