package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.monticar.generator.cpp.resolver.SymTabCreator;
import de.monticore.symboltable.Scope;
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

    private GeneratorCppCli() {
    }

    public static void main(String[] args) {
        Options options = new Options();
        options.addOption(OPTION_MODELS_PATH);
        options.addOption(OPTION_ROOT_MODEL);
        options.addOption(OPTION_OUTPUT_PATH);
        options.addOption(OPTION_FLAG_TESTS);
        CommandLineParser parser = new DefaultParser();
        CommandLine cliArgs;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            System.err.println("argument parsing exception: " + e.getMessage());
            System.exit(1);
            return;
        }
        Path modelsDirPath = Paths.get(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        SymTabCreator symTabCreator = new SymTabCreator(modelsDirPath);
        Scope symtab = symTabCreator.createSymTab();
        Resolver resolver = new Resolver(symtab);
        ExpandedComponentInstanceSymbol componentSymbol = resolver.getExpandedComponentInstanceSymbol(rootModelName).orElse(null);
        if (componentSymbol == null) {
            Log.error("could not resolve component " + rootModelName);
            System.exit(1);
            return;
        }
        GeneratorCPP g = new GeneratorCPP();
        g.setModelsDirPath(modelsDirPath);
        g.setGenerationTargetPath(outputPath);
        g.setGenerateTests(cliArgs.hasOption(OPTION_FLAG_TESTS.getOpt()));
        try {
            g.generateFiles(componentSymbol, symtab);
        } catch (IOException e) {
            Log.error("error during generation", e);
            System.exit(1);
        }
    }
}
