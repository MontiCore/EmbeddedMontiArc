package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.*;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * Usage example: TODO
 * java -cp emam2cpp.jar \
 * de.monticore.lang.monticar.generator.cpp.GeneratorCppCli \
 * --models-dir=C:\Users\vpupkin\proj\src\emam \
 * --root-model=de.rwth.vpupkin.modeling.autopilot.autopilot \
 * --output-dir=C:\Users\vpupkin\proj\target\cpp-gen\autopilot \
 * --flag-generate-tests \
 * --flag-use-armadillo-backend
 * --check-model-dir
 */
public final class DistributedTargetGeneratorCli {

    public static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to directory with EMAM models")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("fully qualified name of the root model")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory for tests")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_GENERATORS = Option.builder("g")
            .longOpt("generators")
            .desc("identifiers for the generators that should be used")
            .hasArg(true)
            .required(true)
            .numberOfArgs(10)
            .valueSeparator(',')
            .build();

    public static final String GENERATOR_CPP = "cpp";
    public static final String GENERATOR_ROSCP = "roscpp";

    public DistributedTargetGeneratorCli() {
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
        options.addOption(OPTION_GENERATORS);
        return options;
    }

    private static Set<String> getGeneratorNames() {
        HashSet<String> res = new HashSet<>();
        res.add(GENERATOR_CPP);
        res.add(GENERATOR_ROSCP);
        return res;
    }

    private static CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        CommandLine cliArgs;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            Log.error("Argument parsing exception", e);
            return null;
        }
        return cliArgs;
    }

    private static void runGenerator(CommandLine cliArgs) {
        String modelsDirPath = expandHomeDir(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));


        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = expandHomeDir(cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt()));
        Set<String> generators = Arrays.stream(cliArgs.getOptionValues(OPTION_GENERATORS.getOpt())).collect(Collectors.toSet());
        TaggingResolver taggingResolver = AbstractSymtab.createSymTabAndTaggingResolver(modelsDirPath);

        DistributedTargetGenerator generator = new DistributedTargetGenerator();
        generator.setGenerationTargetPath(outputPath);

        Set<String> validGenNames = getGeneratorNames();
        generators.forEach(genName -> {
            if (validGenNames.contains(genName)) {
                Log.warn("Using generator " + genName);
            } else {
                Log.warn("Ignoring " + genName + " since it is not a valid generator name.");
            }
        });

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve(rootModelName, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        if (componentInstanceSymbol == null) {
            Log.error("The given component cannot be resolved!");
            return;
        }

        if (generators.contains(GENERATOR_CPP)) {
            generator.add(new CPPGenImpl(), "cpp");
        }

        if (generators.contains(GENERATOR_ROSCP)) {
            generator.add(new RosCppGenImpl(), "roscpp");
            RosToEmamTagSchema.registerTagTypes(taggingResolver);
            TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);
        }

        try {
            generator.generate(componentInstanceSymbol, taggingResolver);
        } catch (IOException e) {
            Log.error("Error generating files", e);
        }

    }

    private static String expandHomeDir(String path) {
        if (path.startsWith("~")) {
            return System.getProperty("user.home") + path.substring(1);
        }
        return path;
    }
}
