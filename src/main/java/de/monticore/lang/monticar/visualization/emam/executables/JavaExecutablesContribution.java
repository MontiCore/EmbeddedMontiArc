/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import org.apache.commons.exec.CommandLine;
import org.apache.commons.exec.DefaultExecutor;

import java.io.IOException;
import java.nio.file.Path;
import java.util.logging.Logger;

public abstract class JavaExecutablesContribution implements ExecutablesContribution {
    protected final Logger logger;
    protected final PathsService pathsService;
    protected final OptionsService optionsService;

    @Inject
    public JavaExecutablesContribution(Logger logger, PathsService pathsService, OptionsService optionsService) {
        this.logger = logger;
        this.pathsService = pathsService;
        this.optionsService = optionsService;
    }

    protected abstract Path getExecutableJAR();

    protected void execute(String[] args) throws IOException {
        CommandLine commandLine = new CommandLine("java");
        DefaultExecutor executor = new DefaultExecutor();
        String jar = this.getExecutableJAR().toString();

        commandLine.addArgument("-jar");
        commandLine.addArgument(jar);
        commandLine.addArguments(args);

        executor.execute(commandLine);
    }
}
