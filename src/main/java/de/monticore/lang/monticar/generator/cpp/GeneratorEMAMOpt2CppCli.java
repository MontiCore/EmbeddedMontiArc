/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.optimizationSolver.solver.Solver;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.cli.*;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

public class GeneratorEMAMOpt2CppCli {

    public static final Option OPTION_MODELS_PATH = Option.builder("m").longOpt("models-dir").desc("full path to directory with EMAM models e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\src\\main\\emam").hasArg(true).required(true).build();
    public static final Option OPTION_ROOT_MODEL = Option.builder("r").longOpt("root-model").desc("fully qualified name of the root model e.g. de.rwth.vpupkin.modeling.mySuperAwesomeAutopilotComponent").hasArg(true).required(true).build();
    public static final Option OPTION_OUTPUT_PATH = Option.builder("o").longOpt("output-dir").desc("full path to output directory for tests e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\target\\gen-cpp").hasArg(true).required(true).build();
    public static final Option OPTION_FLAG_TESTS = Option.builder("t").longOpt("flag-generate-tests").desc("optional flag indicating if tests generation is needed").hasArg(false).required(false).build();
    public static final Option OPTION_FLAG_ALGEBRAIC = Option.builder("a").longOpt("flag-use-algebraic").desc("optional flag indicating if algebraic optimizations should be on").hasArg(false).required(false).build();
    public static final Option OPTION_FLAG_THREADING = Option.builder("a").longOpt("flag-use-threading").desc("optional flag indicating if threading optimizations should be on").hasArg(false).required(false).build();
    public static final Option OPTION_FLAG_AUTOPILOT_ADAPTER = Option.builder().longOpt("flag-generate-autopilot-adapter").desc("optional flag indicating if autopilot adapter should be generated").hasArg(false).required(false).build();
    public static final Option OPTION_FLAG_CHECK_MODEL_DIR = Option.builder().longOpt("check-model-dir").desc("optional flag indicating if model dir should be checked for creation of component and stream list").hasArg(false).required(false).build();
    public static final Option OPTION_FLAG_SERVER_WRAPPER = Option.builder().longOpt("flag-generate-server-wrapper").desc("optional flag indicating if model should be wrapped into a server").hasArg(false).required(false).build();
    public static final Option OPTION_FLAG_GENERATE_CMAKE = Option.builder().longOpt("flag-generate-cmake").desc("optional flag indicating if model should be additionally generate CMake files").hasArg(false).required(false).build();
    public static final Option OPTION_PREFERED_SOLVER = Option.builder("s").longOpt("prefered-solver").desc("prefered solver (IPOPT, CPLEX)").hasArg(true).required(false).build();

    public static final Option OPTION_IPOPT_PRINT_LEVEL = Option.builder().longOpt("print-level").desc("verbosity of IPOPT solver output from 0-12").hasArg(true).required(false).build();
    public static final Option OPTION_IPOPT_MAX_ITER = Option.builder().longOpt("max-iter").desc("maximal number of iterations.").hasArg(true).required(false).build();
    public static final Option OPTION_IPOPT_NUMERIC_TOL = Option.builder().longOpt("numeric-tol").desc("approximate accuracy in first order necessary conditions").hasArg(true).required(false).build();
    public static final Option OPTION_IPOPT_POINT_PERMUTATION = Option.builder().longOpt("point-permutation-radius").desc("maximum amount of random permutation").hasArg(true).required(false).build();

    private GeneratorEMAMOpt2CppCli() {
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
        options.addOption(OPTION_FLAG_AUTOPILOT_ADAPTER);
        options.addOption(OPTION_FLAG_CHECK_MODEL_DIR);
        options.addOption(OPTION_FLAG_SERVER_WRAPPER);
        options.addOption(OPTION_FLAG_ALGEBRAIC);
        options.addOption(OPTION_FLAG_THREADING);
        options.addOption(OPTION_FLAG_GENERATE_CMAKE);
        options.addOption(OPTION_PREFERED_SOLVER);
        return options;
    }

    private static CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        try {
            CommandLine cliArgs = parser.parse(options, args);
            return cliArgs;
        } catch (ParseException var5) {
            System.err.println("argument parsing exception: " + var5.getMessage());
            System.exit(1);
            return null;
        }
    }

    private static void runGenerator(CommandLine cliArgs) {
        Path modelsDirPath = Paths.get(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        TaggingResolver symTab = EMAMOpt2CPPSymbolTableHelper.getInstance().createSymTabAndTaggingResolver(new String[]{modelsDirPath.toString()});
        Resolver resolver = new Resolver(symTab);
        EMAComponentInstanceSymbol componentSymbol = resolveSymbol(resolver, rootModelName);
        GeneratorEMAMOpt2CPP g = new GeneratorEMAMOpt2CPP();
        g.setUseAlgebraicOptimizations(false);
        g.setUseThreadingOptimization(false);
        g.setModelsDirPath(modelsDirPath);
        g.setGenerationTargetPath(outputPath);
        g.setGenerateTests(cliArgs.hasOption(OPTION_FLAG_TESTS.getOpt()));
        g.setCheckModelDir(cliArgs.hasOption(OPTION_FLAG_CHECK_MODEL_DIR.getLongOpt()));
        g.setGenerateServerWrapper(cliArgs.hasOption(OPTION_FLAG_SERVER_WRAPPER.getLongOpt()));
        g.setGenerateAutopilotAdapter(cliArgs.hasOption(OPTION_FLAG_AUTOPILOT_ADAPTER.getLongOpt()));
        g.setUseAlgebraicOptimizations(cliArgs.hasOption(OPTION_FLAG_ALGEBRAIC.getLongOpt()));
        g.setUseThreadingOptimization(cliArgs.hasOption(OPTION_FLAG_THREADING.getLongOpt()));
        g.setGenerateCMake(cliArgs.hasOption(OPTION_FLAG_GENERATE_CMAKE.getLongOpt()));
        setPreferedSolverFromArg(g, cliArgs.getOptionValue(OPTION_PREFERED_SOLVER.getOpt()));

        try {
            if (componentSymbol != null) {
                g.generateFiles(componentSymbol, symTab);
            } else {
                g.saveFilesToDisk(g.handleTestAndCheckDir(symTab, componentSymbol));
            }
        } catch (IOException var9) {
            Log.error("error during generation", var9);
            System.exit(1);
        }

    }

    private static void setPreferedSolverFromArg(GeneratorEMAMOpt2CPP g, String optionValue) {
        if (optionValue != null) {
            if (optionValue.contentEquals("IPOPT")) {
                g.setPreferedSolver(Solver.Cplex);
            } else if (optionValue.contentEquals("CPLEX")) {
                g.setPreferedSolver(Solver.Cplex);
            } else {
                Log.warn(String.format("Solver %s not found. Using default solver instead."));
            }
        }
    }

    private static EMAComponentInstanceSymbol resolveSymbol(Resolver resolver, String rootModelName) {
        EMAComponentInstanceSymbol componentSymbol = (EMAComponentInstanceSymbol) resolver.getExpandedComponentInstanceSymbol(rootModelName).orElse(null);
        if (componentSymbol == null) {
            Log.error("could not resolve component " + rootModelName);
            return null;
        } else {
            return componentSymbol;
        }
    }
}
