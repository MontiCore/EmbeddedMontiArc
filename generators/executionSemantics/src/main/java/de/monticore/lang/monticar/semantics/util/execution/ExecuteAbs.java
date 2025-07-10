/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.execution;

import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;
import org.apache.commons.exec.PumpStreamHandler;
import org.apache.commons.io.output.ByteArrayOutputStream;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Optional;

public abstract class ExecuteAbs {

    public static final String TARGET_TEMP_EXECUTE_DIR = "target/temp/execute";

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

    protected static Optional<File> searchIn(String relativePath, String... directories) {
        File file = getFile(relativePath);
        if (file.exists())
            return Optional.of(file);
        for (String dir : directories) {
            file = getFile(String.join("/", dir, relativePath));
            if (file.exists())
                return Optional.of(file);
        }
        return Optional.empty();
    }

    private static File getFile(String file) {
        Path temp = Paths.get(String.join("/", TARGET_TEMP_EXECUTE_DIR, file));
        try {
            InputStream resourceAsStream = Thread.currentThread().getContextClassLoader().getResourceAsStream(file);
            if (resourceAsStream == null || temp == null) return new File("");
            temp.getParent().toFile().mkdirs();
            Files.copy(resourceAsStream, temp, StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException e) {
            return new File("");
        }

        return temp.toFile();
    }
}
