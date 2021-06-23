/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.semantics.Constants;
import de.monticore.lang.monticar.semantics.util.BasicLibrary;
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
 * --check-model-dir
 */
public final class GeneratorCppCli {

    public static final Option OPTION_MODELS_PATH = Option.builder("m")
            .longOpt("models-dir")
            .desc("full path to directory with EMAM models e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\src\\main\\emam")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_ROOT_MODEL = Option.builder("r")
            .longOpt("root-model")
            .desc("fully qualified name of the root model e.g. de.rwth.vpupkin.modeling.mySuperAwesomeAutopilotComponent")
            .hasArg(true)
            .required(true)
            .build();

    public static final Option OPTION_OUTPUT_PATH = Option.builder("o")
            .longOpt("output-dir")
            .desc("full path to output directory for tests e.g. C:\\Users\\vpupkin\\proj\\MyAwesomeAutopilot\\target\\gen-cpp\n" +
                    "default is: ./target/generated-sources-cpp/")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_OUTPUT_NAME = Option.builder("n")
            .longOpt("output-name")
            .desc("Name for the dynamic-interface or server adapter.")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_TESTS = Option.builder("t")
            .longOpt("flag-generate-tests")
            .desc("optional flag indicating if tests generation is needed")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_ARMADILLO = Option.builder("a")
            .longOpt("flag-use-armadillo-backend")
            .desc("optional flag indicating if Armadillo library should be used as backend")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_IMPORT_ARMADILLO = Option.builder()
            .longOpt("armadillo-import")
            .desc("If enabled, the project will include Armadillo for compilation based on the ARMADILLO_PATH environment variable")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_ALGEBRAIC = Option.builder("a")
            .longOpt("flag-use-algebraic")
            .desc("optional flag indicating if algebraic optimizations should be on")
            .hasArg(false)
            .required(false)
            .build();


    public static final Option OPTION_FLAG_THREADING = Option.builder("a")
            .longOpt("flag-use-threading")
            .desc("optional flag indicating if threading optimizations should be on")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_EXEC_LOGGING = Option.builder("a")
            .longOpt("flag-use-exec-logging")
            .desc("optional flag indicating if execution logging should be on")
            .hasArg(false)
            .required(false)
            .build();


    public static final Option OPTION_FLAG_LIBRARY_INTERFACE = Option.builder()
            .longOpt("library-interface")
            .desc("Enables the autopilot library-adapter generation")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_GEN_TCP_SERVER = Option.builder("tcp")
            .longOpt("tcp-adapter")
            .desc("Generate the TCP-Server adapter for the model")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_CHECK_MODEL_DIR = Option.builder()
            .longOpt("check-model-dir")
            .desc("optional flag indicating if model dir should be checked for creation of component and stream list")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_SERVER_WRAPPER = Option.builder()
            .longOpt("flag-generate-server-wrapper")
            .desc("optional flag indicating if model should be wrapped into a server")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_FLAG_CMAKE = Option.builder()
            .longOpt("flag-generate-cmake")
            .desc("optional flag indicating if a CMake project should be generated to build the model")
            .hasArg(false)
            .required(false)
            .build();

    public static final Option OPTION_DELTA_T = Option.builder("dt")
            .longOpt("time-step")
            .desc("optional parameter to set the time step duration as double")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_ATOL = Option.builder("atol")
            .longOpt("absolute-tolerance")
            .desc("optional parameter to set the absolute tolerance for numeric solves")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_RTOL = Option.builder("rtol")
            .longOpt("relative-tolerance")
            .desc("optional parameter to set the relative tolerance for numeric solves")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_JTOL = Option.builder("jtol")
            .longOpt("jacobean-tolerance")
            .desc("optional parameter to set the tolerance for Jacobeans for numeric solves")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_DT_SOLVER = Option.builder("dts")
            .longOpt("time-step-solver")
            .desc("optional parameter to set the time step for numeric solves as double")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_RESOLVE_LOOPS = Option.builder()
            .longOpt("resolve-loops")
            .desc("optional parameter to resolve loops automatically,...")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_HANDLE_ARTIFICIAL = Option.builder()
            .longOpt("handle-artificial")
            .desc("optional parameter to handle artificial loops automatically,...")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_SOLVE_LOOPS_SYMBOLIC = Option.builder()
            .longOpt("solve-loops-symbolic")
            .desc("optional parameter to solve loops analytically,...")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_SOLVE_SPECIFICATION_SYMBOLIC = Option.builder()
            .longOpt("solve-specifications-symbolic")
            .desc("optional parameter to solve specifications analytically,...")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_WARN_LOOPS = Option.builder()
            .longOpt("warn-loops")
            .desc("optional parameter to log loops")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_WARN_ARTIFICIAL_LOOPS = Option.builder()
            .longOpt("warn-artificial-loops")
            .desc("optional parameter to log artificial loops")
            .hasArg(true)
            .required(false)
            .build();

    public static final Option OPTION_LOG_SYMBOLIC_SOLVE = Option.builder()
            .longOpt("log-symbolic-solve")
            .desc("optional parameter to log symbolic solves")
            .hasArg(true)
            .required(false)
            .build();

    private GeneratorCppCli() {
    }

    public static void main(String[] args) {
        //Log.initDEBUG();
        Options options = getOptions();
        CommandLineParser parser = new DefaultParser();
        CommandLine cliArgs = parseArgs(options, parser, args);
        if (cliArgs != null) {
            runGenerator(cliArgs);
        }else{
            Log.error("Error parsing cli arguments!");
            //System.exit(1);
        }
    }

    public static Options getOptions() {
        Options options = new Options();
        addBaseOptions(options);
        addEMAM2CPPOptions(options);
        return options;
    }

    public static void addBaseOptions(Options options) {
        options.addOption(OPTION_MODELS_PATH);
        options.addOption(OPTION_ROOT_MODEL);
        options.addOption(OPTION_OUTPUT_PATH);
        options.addOption(OPTION_FLAG_CMAKE);
    }

    public static void addEMAM2CPPOptions(Options options) {
        options.addOption(OPTION_FLAG_TESTS);
        options.addOption(OPTION_FLAG_ARMADILLO);
        options.addOption(OPTION_IMPORT_ARMADILLO);
        options.addOption(OPTION_FLAG_LIBRARY_INTERFACE);
        options.addOption(OPTION_OUTPUT_NAME);
        options.addOption(OPTION_FLAG_GEN_TCP_SERVER);
        options.addOption(OPTION_FLAG_CHECK_MODEL_DIR);
        options.addOption(OPTION_FLAG_SERVER_WRAPPER);
        options.addOption(OPTION_FLAG_ALGEBRAIC);
        options.addOption(OPTION_FLAG_THREADING);
        options.addOption(OPTION_FLAG_EXEC_LOGGING);
        options.addOption(OPTION_DELTA_T);
        options.addOption(OPTION_ATOL);
        options.addOption(OPTION_RTOL);
        options.addOption(OPTION_JTOL);
        options.addOption(OPTION_DT_SOLVER);
        options.addOption(OPTION_RESOLVE_LOOPS);
        options.addOption(OPTION_HANDLE_ARTIFICIAL);
        options.addOption(OPTION_SOLVE_LOOPS_SYMBOLIC);
        options.addOption(OPTION_SOLVE_SPECIFICATION_SYMBOLIC);
        options.addOption(OPTION_WARN_LOOPS);
        options.addOption(OPTION_WARN_ARTIFICIAL_LOOPS);
        options.addOption(OPTION_LOG_SYMBOLIC_SOLVE);
    }

    public static CommandLine parseArgs(Options options, CommandLineParser parser, String[] args) {
        CommandLine cliArgs = null;
        try {
            cliArgs = parser.parse(options, args);
        } catch (ParseException e) {
            System.err.println("argument parsing exception: " + e.getMessage());
            //System.exit(1);
            //return null;
        }
        return cliArgs;
    }

    public static void runGenerator(CommandLine cliArgs) {
        Path modelsDirPath = Paths.get(cliArgs.getOptionValue(OPTION_MODELS_PATH.getOpt()));
        String rootModelName = cliArgs.getOptionValue(OPTION_ROOT_MODEL.getOpt());
        String outputPath = cliArgs.getOptionValue(OPTION_OUTPUT_PATH.getOpt());
        TaggingResolver symTab = getSymTabAndTaggingResolver(modelsDirPath);
        Resolver resolver = new Resolver(symTab);
        EMAComponentInstanceSymbol componentSymbol = resolveSymbol(resolver, rootModelName);

        GeneratorCPP g = new GeneratorCPP();
        g.setUseAlgebraicOptimizations(false);
        g.setUseThreadingOptimization(false);
        g.setModelsDirPath(modelsDirPath);
        g.setGenerationTargetPath(outputPath);
        g.setGenerateTests(cliArgs.hasOption(OPTION_FLAG_TESTS.getOpt()));
        g.setImportArmadillo(cliArgs.hasOption(OPTION_IMPORT_ARMADILLO.getLongOpt()));
        if (cliArgs.hasOption(OPTION_FLAG_ARMADILLO.getOpt())) {
            g.useArmadilloBackend();
        }
        g.setCheckModelDir(cliArgs.hasOption(OPTION_FLAG_CHECK_MODEL_DIR.getLongOpt()));
        g.setGenerateServerWrapper(cliArgs.hasOption(OPTION_FLAG_SERVER_WRAPPER.getLongOpt()));
        g.setGenerateLibraryInterface(cliArgs.hasOption(OPTION_FLAG_LIBRARY_INTERFACE.getLongOpt()));
        g.setGenerateServerAdapter(cliArgs.hasOption(OPTION_FLAG_GEN_TCP_SERVER.getLongOpt()));
        g.setOutputName(cliArgs.getOptionValue(OPTION_OUTPUT_NAME.getOpt()));

        g.setUseAlgebraicOptimizations(cliArgs.hasOption(OPTION_FLAG_ALGEBRAIC.getLongOpt()));
        g.setUseThreadingOptimization(cliArgs.hasOption(OPTION_FLAG_THREADING.getLongOpt()));
        g.setExecutionLoggingActive(cliArgs.hasOption(OPTION_FLAG_EXEC_LOGGING.getLongOpt()));
        g.setGenerateCMake(cliArgs.hasOption(OPTION_FLAG_CMAKE.getLongOpt()));

        if (cliArgs.hasOption(OPTION_DELTA_T.getOpt()))
            g.setDeltaT(cliArgs.getOptionValue(OPTION_DELTA_T.getOpt()));
        if (cliArgs.hasOption(OPTION_DELTA_T.getLongOpt()))
            g.setDeltaT(cliArgs.getOptionValue(OPTION_DELTA_T.getLongOpt()));

        if (cliArgs.hasOption(OPTION_ATOL.getOpt()))
            g.setATol(cliArgs.getOptionValue(OPTION_ATOL.getOpt()));
        if (cliArgs.hasOption(OPTION_ATOL.getLongOpt()))
            g.setATol(cliArgs.getOptionValue(OPTION_ATOL.getLongOpt()));

        if (cliArgs.hasOption(OPTION_RTOL.getOpt()))
            g.setRTol(cliArgs.getOptionValue(OPTION_RTOL.getOpt()));
        if (cliArgs.hasOption(OPTION_RTOL.getLongOpt()))
            g.setRTol(cliArgs.getOptionValue(OPTION_RTOL.getLongOpt()));

        if (cliArgs.hasOption(OPTION_JTOL.getOpt()))
            g.setJTol(cliArgs.getOptionValue(OPTION_JTOL.getOpt()));
        if (cliArgs.hasOption(OPTION_JTOL.getLongOpt()))
            g.setJTol(cliArgs.getOptionValue(OPTION_JTOL.getLongOpt()));

        if (cliArgs.hasOption(OPTION_DT_SOLVER.getOpt()))
            g.setDeltaTSolver(cliArgs.getOptionValue(OPTION_DT_SOLVER.getOpt()));
        if (cliArgs.hasOption(OPTION_DT_SOLVER.getLongOpt()))
            g.setDeltaTSolver(cliArgs.getOptionValue(OPTION_DT_SOLVER.getLongOpt()));


        if (cliArgs.hasOption(OPTION_RESOLVE_LOOPS.getLongOpt()))
            g.setResolveLoops(cliArgs.getOptionValue(OPTION_RESOLVE_LOOPS.getLongOpt()));

        if (cliArgs.hasOption(OPTION_HANDLE_ARTIFICIAL.getLongOpt()))
            g.setHandleArtificial(cliArgs.getOptionValue(OPTION_HANDLE_ARTIFICIAL.getLongOpt()));

        if (cliArgs.hasOption(OPTION_SOLVE_LOOPS_SYMBOLIC.getLongOpt()))
            g.setSolveLoopsSymbolic(cliArgs.getOptionValue(OPTION_SOLVE_LOOPS_SYMBOLIC.getLongOpt()));

        if (cliArgs.hasOption(OPTION_SOLVE_SPECIFICATION_SYMBOLIC.getLongOpt()))
            g.setSolveSpecificationSymbolic(cliArgs.getOptionValue(OPTION_SOLVE_SPECIFICATION_SYMBOLIC.getLongOpt()));

        if (cliArgs.hasOption(OPTION_WARN_LOOPS.getLongOpt()))
            g.setWarnLoops(cliArgs.getOptionValue(OPTION_WARN_LOOPS.getLongOpt()));

        if (cliArgs.hasOption(OPTION_WARN_ARTIFICIAL_LOOPS.getLongOpt()))
            g.setWarnArtificial(cliArgs.getOptionValue(OPTION_WARN_ARTIFICIAL_LOOPS.getLongOpt()));

        if (cliArgs.hasOption(OPTION_LOG_SYMBOLIC_SOLVE.getLongOpt()))
            g.setLogSymbolicSolve(cliArgs.getOptionValue(OPTION_LOG_SYMBOLIC_SOLVE.getLongOpt()));



        try {
            if (componentSymbol != null) {
                g.generateFiles(symTab, componentSymbol);
            } else {
                g.saveFilesToDisk(g.handleTestAndCheckDir(symTab, componentSymbol));
            }
        } catch (IOException e) {
            Log.error("error during generation", e);
            //System.exit(1);
        }

    }

    private static TaggingResolver getSymTabAndTaggingResolver(Path modelsDirPath) {
        BasicLibrary.extract();
        return AbstractSymtab.createSymTabAndTaggingResolver(modelsDirPath.toString(),
                Constants.SYNTHESIZED_COMPONENTS_ROOT, BasicLibrary.BASIC_LIBRARY_ROOT);
    }

    private static EMAComponentInstanceSymbol resolveSymbol(Resolver resolver, String rootModelName) {
        EMAComponentInstanceSymbol componentSymbol = resolver.getExpandedComponentInstanceSymbol(rootModelName).orElse(null);
        if (componentSymbol == null) {
            Log.error("could not resolve component " + rootModelName);
            //System.exit(1);
            return null;
        }
        return componentSymbol;
    }
}
