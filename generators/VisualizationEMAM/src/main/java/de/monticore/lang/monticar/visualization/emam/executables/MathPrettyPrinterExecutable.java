/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import org.apache.commons.cli.ParseException;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

public class MathPrettyPrinterExecutable extends JavaExecutablesContribution {
    protected boolean internal;
    protected String modelPath;
    protected String outputPath;

    @Inject
    public MathPrettyPrinterExecutable(Logger logger, PathsService pathsService, OptionsService optionsService) {
        super(logger, pathsService, optionsService);
    }

    protected String[] buildArguments() {
        List<String> arguments = new ArrayList<>();

        Preconditions.checkNotNull(this.modelPath);
        Preconditions.checkNotNull(this.outputPath);

        if (this.internal) arguments.add("-i");

        arguments.add("-mp");
        arguments.add(this.modelPath);
        arguments.add("-out");
        arguments.add(this.outputPath);

        return arguments.toArray(new String[0]);
    }

    @Override
    public void prepare() {
        Path outputPath = this.optionsService.getOptionAsPath("out");

        this.internal = true;
        this.modelPath = this.optionsService.getOptionAsString("mp");
        this.outputPath = outputPath.resolve("mathviews").toString();
    }

    @Override
    public void execute() throws IOException {
        String[] arguments = this.buildArguments();

        this.logger.info("Executing MathPrettyPrinter...");
        super.execute(arguments);
        this.logger.info("...MathPrettyPrinter executed.");
    }

    @Override
    protected Path getExecutableJAR() {
        return this.pathsService.getPath("math-pretty-printer.jar");
    }
}
