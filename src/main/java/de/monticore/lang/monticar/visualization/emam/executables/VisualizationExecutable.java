/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import com.google.common.base.Preconditions;
import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import org.apache.commons.cli.ParseException;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;

public class VisualizationExecutable extends JavaExecutablesContribution {
    protected String input;
    protected String modelPath;
    protected String outputPath;
    protected boolean recursiveDrawing;

    @Inject
    public VisualizationExecutable(Logger logger, PathsService pathsService, OptionsService optionsService) {
        super(logger, pathsService, optionsService);
    }

    protected String[] buildArguments() {
        List<String> arguments = new ArrayList<>();

        Preconditions.checkNotNull(this.input);
        Preconditions.checkNotNull(this.modelPath);
        Preconditions.checkNotNull(this.outputPath);

        if (this.recursiveDrawing) {
            arguments.add("--recursiveDrawing");
            arguments.add("true");
        }

        arguments.add("--input");
        arguments.add(this.input);
        arguments.add("--modelPath");
        arguments.add(this.modelPath);
        arguments.add("--outputPath");
        arguments.add(this.outputPath);

        return arguments.toArray(new String[0]);
    }

    @Override
    public void prepare() {
        Path outputPath = this.optionsService.getOptionAsPath("out");

        this.input = this.optionsService.getOptionAsString("m");
        this.modelPath = this.optionsService.getOptionAsString("mp");
        this.outputPath = outputPath.resolve("visualization").toString() + File.separator;
        this.recursiveDrawing = true;
    }

    @Override
    public void execute() throws IOException {
        String[] arguments = this.buildArguments();

        this.logger.info("Executing Visualization...");
        super.execute(arguments);
        this.logger.info("...Visualization executed.");
    }

    @Override
    protected Path getExecutableJAR() {
        return this.pathsService.getPath("visualization.jar");
    }
}
