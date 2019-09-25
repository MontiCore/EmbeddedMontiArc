/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import com.google.common.flogger.FluentLogger;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.ls.ServerLauncherContribution;
import org.apache.commons.cli.*;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

@Singleton
public class DefaultOptionsService implements OptionsService, ServerLauncherContribution {
    protected final FluentLogger logger;
    protected final OptionsRegistry registry;

    protected CommandLine commandLine;

    @Inject
    protected DefaultOptionsService(OptionsRegistry registry) {
        this.logger = FluentLogger.forEnclosingClass();
        this.registry = registry;
    }

    @Override
    public boolean hasOption(String option) {
        return this.commandLine.hasOption(option);
    }

    @Override
    public String getOptionAsString(String option) {
        return this.commandLine.getOptionValue(option);
    }

    @Override
    public String getOptionAsString(String option, String defaultValue) {
        return this.commandLine.getOptionValue(option, defaultValue);
    }

    @Override
    public Path getOptionAsPath(String option) {
        return Paths.get(this.commandLine.getOptionValue(option));
    }

    @Override
    public Path getOptionAsPath(String option, String defaultPath) {
        return Paths.get(this.commandLine.getOptionValue(option, defaultPath));
    }

    @Override
    public File getOptionAsFile(String option) {
        return Paths.get(this.commandLine.getOptionValue(option)).toFile();
    }

    @Override
    public File getOptionAsFile(String option, String defaultFile) {
        return Paths.get(this.commandLine.getOptionValue(option, defaultFile)).toFile();
    }

    @Override
    public int getOptionAsInteger(String option) {
        return Integer.parseInt(this.commandLine.getOptionValue(option));
    }

    @Override
    public int getOptionAsInteger(String option, String defaultValue) {
        return Integer.parseInt(this.commandLine.getOptionValue(option, defaultValue));
    }

    @Override
    public void onConfigure(String[] arguments) throws ParseException {
        CommandLineParser parser = new DefaultParser();
        Options options = this.registry.getOptions();

        this.logger.atInfo().log("Parsing command line...");

        this.commandLine = parser.parse(options, arguments);
    }
}
