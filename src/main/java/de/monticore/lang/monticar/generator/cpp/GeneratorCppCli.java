package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.monticar.generator.cpp.resolver.SymTabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.CommandLineParser;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Usage example:
 * java -cp emam2cpp.jar \
 * de.monticore.lang.monticar.generator.cpp.GeneratorCppCli \
 * --models-dir=C:\Users\vpupkin\proj\src\emam \
 * --root-model=de.rwth.vpupkin.modeling.autopilot.autopilot \
 * --output-dir=C:\Users\vpupkin\proj\target\cpp-gen\autopilot \
 * --flag-generate-tests \
 * --flag-use-armadillo-backend
 */
public final class GeneratorCppCli {

    private static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to directory with EMAM models e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\src\\main\\emam")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("fully qualified name of the root model e.g. de.rwth.vpupkin.modeling.mySuperAwesomeAutopilotComponent")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory for tests e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\target\\gen-cpp")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_FLAG_TESTS = Option.builder("t")
            .longOpt("flag-generate-tests")
            .desc("optional flag indicating if tests generation is needed")
            .hasArg(false)
            .required(false)
            .build();

    private static final Option OPTION_FLAG_ARMADILLO = Option.builder("a")
            .longOpt("flag-use-armadillo-backend")
            .desc("optional flag indicating if Armadillo library should be used as backend")
            .hasArg(false)
            .required(false)
            .build();

    private GeneratorCppCli() {
    }

    public static void main(String[] args) {
        Options options = getOptions();
        CommandLineParser parser = new DefaultParser();
        CommandLine cliArgs = parseArgs(options, parser, args);
        if (cliArgs != null) {
            runGenerator(cliArgs);
        }
    }

    private static Options getOptions() {
        Options options = new Options();
        options.addOption(OPTION_MODELS_PATH);
        options.addOption(OPTION_ROOT_MODEL);
        options.addOption(OPTION_OUTPUT_PATH);
        options.addOption(OPTION_FLAG_TESTS);
        options.addOption(OPTION_FLAG_ARMADILLO);
        return options;
    }

    private static CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        CommandLine cliArgs;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            System.err.println("argument parsing exception: " + e.getMessage());
            System.exit(1);
            return null;
        }
        return cliArgs;
    }

    private static void runGenerator(CommandLine cliArgs) {
        Path modelsDirPath = Paths.get(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        TaggingResolver symTab = getSymTab(modelsDirPath);
        Resolver resolver = new Resolver(symTab);
        ExpandedComponentInstanceSymbol componentSymbol = resolveSymbol(resolver, rootModelName);
        if (componentSymbol != null) {
            GeneratorCPP g = new GeneratorCPP();
            g.setModelsDirPath(modelsDirPath);
            g.setGenerationTargetPath(outputPath);
            g.setGenerateTests(cliArgs.hasOption(OPTION_FLAG_TESTS.getOpt()));
            if (cliArgs.hasOption(OPTION_FLAG_ARMADILLO.getOpt())) {
                g.useArmadilloBackend();
            }
            try {
                g.generateFiles(symTab, componentSymbol, symTab);
            } catch (IOException e) {
                Log.error("error during generation", e);
                System.exit(1);
            }
        }
    }

    private static TaggingResolver getSymTab(Path modelsDirPath) {
        SymTabCreator symTabCreator = new SymTabCreator(modelsDirPath);
        return symTabCreator.createSymTabAndTaggingResolver();
    }

    private static ExpandedComponentInstanceSymbol resolveSymbol(Resolver resolver, String rootModelName) {
        ExpandedComponentInstanceSymbol componentSymbol = resolver.getExpandedComponentInstanceSymbol(rootModelName).orElse(null);
        if (componentSymbol == null) {
            Log.error("could not resolve component " + rootModelName);
            System.exit(1);
            return null;
        }
        return componentSymbol;
    }
}
