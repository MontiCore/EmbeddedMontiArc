/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.*;

import java.nio.file.Path;
import java.nio.file.Paths;

/**
 *
 */
public class GenericCNNArchCli {
    private final CNNArchGenerator cnnArchGenerator;

    public static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to the directory with the CNNArch model")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("name of the architecture")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory for tests")
            .hasArg(true)
            .required(false)
            .build();

    public GenericCNNArchCli(CNNArchGenerator cnnArchGenerator) {
        this.cnnArchGenerator = cnnArchGenerator;
    }

    public void run(String[] args) {
        Options options = getOptions();
        CommandLineParser parser = new DefaultParser();
        CommandLine cliArgs = parseArgs(options, parser, args);
        if (cliArgs != null) {
            runGenerator(cliArgs);
        }
    }

    private Options getOptions() {
        Options options = new Options();
        options.addOption(OPTION_MODELS_PATH);
        options.addOption(OPTION_ROOT_MODEL);
        options.addOption(OPTION_OUTPUT_PATH);
        return options;
    }

    private CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        CommandLine cliArgs;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            Log.error("argument parsing exception: " + e.getMessage());
            quitGeneration();
            return null;
        }
        return cliArgs;
    }

    private void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    private void runGenerator(CommandLine cliArgs) {
        Path modelsDirPath = Paths.get(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        if (outputPath != null){
            cnnArchGenerator.setGenerationTargetPath(outputPath);
        }
        cnnArchGenerator.generate(modelsDirPath, rootModelName);
    }
}
