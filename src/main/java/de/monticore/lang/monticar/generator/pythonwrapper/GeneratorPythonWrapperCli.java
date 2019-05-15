package de.monticore.lang.monticar.generator.pythonwrapper;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.*;

import java.io.IOException;

/**
 *
 */
class GeneratorPythonWrapperCli {
    private static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to the directory with the EMADL or EMAM model")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("name of the root model")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory")
            .hasArg(true)
            .required(false)
            .build();

    public void run(String[] args) {
        Options options = setupCliOptions();
        CommandLine parsedCliArguments = parseCliArguments(args, options);

        assert parsedCliArguments != null;
        final String modelPath = getModelPathFromParsedArguments(parsedCliArguments);
        final String rootModel = getRootModelFromParsedArguments(parsedCliArguments);
        final String outputPath = getOutputPathFromParsedArguments(parsedCliArguments);

        GeneratorPythonWrapperStandaloneApi api = new GeneratorPythonWrapperStandaloneApi();
        api.generate(modelPath, rootModel, outputPath);
    }

    private String getOutputPathFromParsedArguments(CommandLine parsedCliArguments) {
        String outputPath = parsedCliArguments.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        if (outputPath == null) {
            outputPath = getModelPathFromParsedArguments(parsedCliArguments);
        }
        return outputPath;
    }

    private String getRootModelFromParsedArguments(CommandLine parsedCliArguments) {
        final String rootModel = parsedCliArguments.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        if (rootModel == null) {
            logAndThrowIllegalArgumentException("No root model given but it is required");
        }
        return rootModel;
    }

    private String getModelPathFromParsedArguments(CommandLine parsedCliArguments) {
        final String modelPath = parsedCliArguments.getOptionValue(OPTION_MODELS_PATH.getOpt());
        if (modelPath == null) {
            logAndThrowIllegalArgumentException("No model path was given but it is required");
        }
        return modelPath;
    }

    private CommandLine parseCliArguments(String[] args, Options options) {
        CommandLineParser commandLineParser = new DefaultParser();

        CommandLine parsedCliArguments = null;
        try {
            parsedCliArguments = commandLineParser.parse(options, args);

        } catch (ParseException e) {
            logAndThrowIllegalArgumentException(e.getMessage());
        }
        return parsedCliArguments;
    }

    private Options setupCliOptions() {
        Options options = new Options();
        options.addOption(OPTION_MODELS_PATH);
        options.addOption(OPTION_ROOT_MODEL);
        options.addOption(OPTION_OUTPUT_PATH);
        return options;
    }

    private void logAndThrowIllegalArgumentException(final String message) {
        Log.error(message);
        throw new IllegalArgumentException(message);
    }

    public static void main(String[] args) {
        GeneratorPythonWrapperCli generatorCli = new GeneratorPythonWrapperCli();
        generatorCli.run(args);
    }
}
