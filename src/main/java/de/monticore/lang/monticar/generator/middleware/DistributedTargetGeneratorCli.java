package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab;
import de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.EMADLGeneratorImpl;
import de.monticore.lang.monticar.generator.middleware.impls.ODVGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * java -cp target/embedded-montiarc-math-middleware-generator-0.0.1-SNAPSHOT-jar-with-dependencies.jar \
 * de.monticore.lang.monticar.generator.middleware.DistributedTargetGeneratorCli \
 * --models-dir=src/test/resources/ \
 * --root-model=tests.a.addComp \
 * --output-dir=target/cli-test/src \
 * --generators=cpp,roscpp
 */
public final class DistributedTargetGeneratorCli {

    private static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to directory with EMAM models")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("fully qualified name of the root model")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory for generated files")
            .hasArg(true)
            .required(true)
            .build();

    private static final Option OPTION_GENERATORS = Option.builder("g")
            .longOpt("generators")
            .desc("identifiers for the generators that should be used")
            .hasArg(true)
            .required(true)
            .numberOfArgs(10)
            .valueSeparator(',')
            .build();

    private static final Option OPTION_EMADL_BACKEND = EMADLGeneratorCli.OPTION_BACKEND;

    public static final String GENERATOR_CPP = "cpp";
    public static final String GENERATOR_EMADL = "emadlcpp";
    public static final String GENERATOR_ROSCPP = "roscpp";
    public static final String GENERATOR_ODV = "odv";

    private DistributedTargetGeneratorCli() {}

    public static void main(String[] args) {
        System.out.println(Arrays.toString(args));

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
        options.addOption(OPTION_EMADL_BACKEND);
        return options;
    }

    private static Set<String> getGeneratorNames() {
        HashSet<String> res = new HashSet<>();
        res.add(GENERATOR_CPP);
        res.add(GENERATOR_EMADL);
        res.add(GENERATOR_ROSCPP);
        res.add(GENERATOR_ODV);
        return res;
    }

    private static CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        CommandLine cliArgs;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            Log.error("0x9A1AC: Argument parsing exception", e);
            return null;
        }
        return cliArgs;
    }

    private static void runGenerator(CommandLine cliArgs) {
        String modelsDirPath = expandHomeDir(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        if(!Files.isDirectory(Paths.get(modelsDirPath))){
            Log.error("0x6444B: Models dir does not exist: " + modelsDirPath);
            return;
        }

        String outputPath = expandHomeDir(cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        Set<String> generators = Arrays.stream(cliArgs.getOptionValues(OPTION_GENERATORS.getOpt())).collect(Collectors.toSet());
        TaggingResolver taggingResolver;
        if (generators.contains(GENERATOR_EMADL)) {
            taggingResolver = EMADLAbstractSymtab.createSymTabAndTaggingResolver(modelsDirPath);
        }
        else{
            taggingResolver = AbstractSymtab.createSymTabAndTaggingResolver(modelsDirPath);
        }

        DistributedTargetGenerator generator = new DistributedTargetGenerator();
        generator.setGenerationTargetPath(outputPath);

        Set<String> validGenNames = getGeneratorNames();
        generators.forEach(genName -> {
            if (validGenNames.contains(genName)) {
                Log.warn("Using generator " + genName);
            } else {
                Log.error("0xE28B6: Not a valid generator Name:" + genName  +".");
            }
        });

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve(rootModelName, ExpandedComponentInstanceSymbol.KIND).orElse(null);

        if (componentInstanceSymbol == null) {
            Log.error("0x5FFAE: The given component cannot be resolved.");
            return;
        }

        if (generators.contains(GENERATOR_CPP)) {
            generator.add(new CPPGenImpl(), "cpp");
        }

        if (generators.contains(GENERATOR_EMADL)) {
            String backendString = cliArgs.getOptionValue(OPTION_EMADL_BACKEND.getOpt());
            generator.add(new EMADLGeneratorImpl(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()), backendString), "cpp");
        }

        if (generators.contains(GENERATOR_ROSCPP)) {
            generator.add(new RosCppGenImpl(), "roscpp");
            RosToEmamTagSchema.registerTagTypes(taggingResolver);
            TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);
        }

        if (generators.contains(GENERATOR_ODV)) {
            generator.add(new ODVGenImpl(), "odv");
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
