/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.ComponentScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cmake.CMakeConfig;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.Dynamics.DynamicHelper;
import de.monticore.lang.monticar.generator.cpp.Dynamics.EventPortValueCheck;
import de.monticore.lang.monticar.generator.cpp.converter.*;
import de.monticore.lang.monticar.generator.cpp.loopSolver.CPPEquationSystemHelper;
import de.monticore.lang.monticar.generator.cpp.loopSolver.EquationSystemComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.loopSolver.NumericSolverOptions;
import de.monticore.lang.monticar.generator.cpp.loopSolver.RHSComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.mathopt.MathOptSolverConfig;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.AutopilotAdapterDataModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ServerWrapperViewModel;
import de.monticore.lang.monticar.generator.testing.StreamTestGenerator;
import de.monticore.lang.monticar.semantics.ExecutionSemantics;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.LoopComponentInstanceSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

/**
 *
 */
public class GeneratorCPP implements EMAMGenerator {
    public static GeneratorCPP currentInstance;
    private Path modelsDirPath;
    private boolean isGenerateTests = false;
    private boolean isGenerateAutopilotAdapter = false;
    private boolean isGenerateServerWrapper = false;
    protected boolean isExecutionLoggingActive = false;
    private final List<EMAMBluePrintCPP> bluePrints = new ArrayList<>();

    protected String generationTargetPath = "./target/generated-sources-cpp/";

    protected boolean algebraicOptimizations = false;
    protected boolean threadingOptimizations = false;
    protected boolean MPIDefinitionFix = true;
    protected MathCommandRegister mathCommandRegister;
    protected boolean generateMainClass = false;
    protected boolean generateSimulatorInterface = false;
    protected boolean checkModelDir = false;
    protected boolean streamTestGenerationMode = false;
    public boolean isGenerateCV = false;

    // CMake
    private boolean generateCMake = false;
    private CMakeConfig cMakeConfig;


    //MathOpt
    private MathOptSolverConfig mathOptSolverConfig = new MathOptSolverConfig();
    private OptimizationSymbolHandler mathOptExecuteMethodGenerator = new OptimizationSymbolHandler();
    private MathOptFunctionFixer mathOptFunctionFixer = new MathOptFunctionFixer();
    private EMAComponentInstanceSymbol rootModel = null;

    // EMAM
    private EMAMSymbolHandler specificationSymbolHandler = new EMAMSymbolHandler();
    private EMAMFunctionFixer emamFunctionFixer = new EMAMFunctionFixer();

    public GeneratorCPP() {
        this.mathCommandRegister = new MathCommandRegisterCPP();
        setGenerateCMake(true);
        useArmadilloBackend();
        TypeConverter.clearTypeSymbols();
        currentInstance = this;

        mathOptExecuteMethodGenerator.setSuccessor(ExecuteMethodGenerator.getInstance());
        mathOptFunctionFixer.setSuccessor(MathFunctionFixer.getInstance());

        specificationSymbolHandler.setSuccessor(mathOptExecuteMethodGenerator);
        emamFunctionFixer.setSuccessor(mathOptFunctionFixer);
    }


    public boolean isExecutionLoggingActive() {
        return isExecutionLoggingActive;
    }

    public void setExecutionLoggingActive(boolean executionLoggingActive) {
        isExecutionLoggingActive = executionLoggingActive;
    }

    protected void setupCMake() {
        cMakeConfig = new CMakeConfig("");
        if (usesArmadilloBackend()) {
            // add dependency on module Armadillo
            cMakeConfig.addModuleDependency(new CMakeFindModule("Armadillo", true));
        }
    }

    public void useArmadilloBackend() {
        MathConverter.curBackend = new ArmadilloBackend();
        setupCMake();
    }

    public boolean usesArmadilloBackend() {
        return MathConverter.curBackend instanceof ArmadilloBackend;
    }

    protected String testNamePostFix = "";
    protected int amountTickValues = 100;

    public void useStreamTestTestGeneration(String testNamePostFix, int amountTickValues) {
        streamTestGenerationMode = true;
        this.testNamePostFix = testNamePostFix;
        this.amountTickValues = amountTickValues;
    }

    public void useStreamTestTestGeneration(String testNamePostFix) {
        useStreamTestTestGeneration(testNamePostFix, 100);
    }

    public void useOctaveBackend() {
        MathConverter.curBackend = new OctaveBackend();
        setupCMake();
        //Log.warn("This backend has been deprecated. Armadillo is the recommended backend now.");
    }

    public String generateString(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol) {
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentInstanceSymbol, taggingResolver);
        return generateString(taggingResolver, componentInstanceSymbol, mathSymbol);
    }

    @Override
    public CMakeConfig getCmakeConfig() {
        if (cMakeConfig == null)
            setupCMake();
        return cMakeConfig;
    }

    @Override
    public boolean isGenerateCMake() {
        return generateCMake;
    }

    @Override
    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    @Override
    public void setGenerationTargetPath(String newPath) {
        this.generationTargetPath = newPath;
    }

    public String generateString(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentSymbol,
                                 MathStatementsSymbol mathStatementsSymbol) {
        StreamTestGenerator streamTestGenerator = new StreamTestGenerator();//only used when creating streamTestsForAComponent
        LanguageUnitCPP languageUnitCPP = new LanguageUnitCPP();
        languageUnitCPP.setGeneratorCPP(this);
        languageUnitCPP.addSymbolToConvert(componentSymbol);
        if (mathStatementsSymbol != null)
            languageUnitCPP.addSymbolToConvert(mathStatementsSymbol);
        if (!streamTestGenerationMode)
            languageUnitCPP.generateBluePrints();
        else
            streamTestGenerator.createStreamTest(componentSymbol, amountTickValues, testNamePostFix);
        EMAMBluePrintCPP bluePrintCPP = null;
        for (EMAMBluePrint bluePrint : languageUnitCPP.getBluePrints()) {
            if (bluePrint.getOriginalSymbol().equals(componentSymbol)) {
                bluePrintCPP = (EMAMBluePrintCPP) bluePrint;
            }
        }

        if (bluePrintCPP != null) {
            bluePrints.add(bluePrintCPP);
            if (componentSymbol.equals(this.rootModel) && ExecutionStepperHelper.isUsed()) {
                Optional<Method> execute = bluePrintCPP.getMethod("execute");
                execute.get().getInstructions().add(new TargetCodeInstruction("advanceTime();\n"));
                bluePrintCPP.addAdditionalUserIncludeStrings(ExecutionStepperHelper.FILENAME);
            }
        }
        String result;
        if (!streamTestGenerationMode)
            result = languageUnitCPP.getGeneratedHeader(taggingResolver, bluePrintCPP);
        else
            result = streamTestGenerator.getCurrentGeneratedStreamTest().toString();
        return result;
    }

    public List<FileContent> currentFileContentList = new ArrayList<>();

    private static Set<EMAMEquationSymbol> equationSystemsAlreadyBuild = new HashSet<>();

    @Override
    public List<FileContent> generateStrings(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentInstanceSymbol) {
        List<FileContent> fileContents = new ArrayList<>();
        if (componentInstanceSymbol.getFullName().equals("simulator.mainController")) {
            setGenerateSimulatorInterface(true);
        } else {
            //setGenerateMainClass(true);
        }

//        ImplementExecutionOrder.exOrder(taggingResolver, componentInstanceSymbol);

        String lastNameWithoutArrayPart = "";
        if (!streamTestGenerationMode) {
            for (EMAComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {
                //fileContents.add(new FileContent(generateString(instanceSymbol, symtab), instanceSymbol));
                int arrayBracketIndex = instanceSymbol.getName().indexOf("[");
                boolean generateComponentInstance = true;
                if (arrayBracketIndex != -1) {
                    generateComponentInstance = !instanceSymbol.getName().substring(0, arrayBracketIndex).equals(lastNameWithoutArrayPart);
                    lastNameWithoutArrayPart = instanceSymbol.getName().substring(0, arrayBracketIndex);
                    Log.info(lastNameWithoutArrayPart, "Without:");
                    Log.info(generateComponentInstance + "", "Bool:");
                }
                if (generateComponentInstance) {
                    fileContents.addAll(generateStrings(taggingResolver, instanceSymbol));
                }
            }
            if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
                fileContents.add(OctaveHelper.getOctaveHelperFileContent());
            if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend")) {
                fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent(isGenerateTests));
                if (ConversionHelper.isUsedCV()) {
                    fileContents.add(ConversionHelper.getConversionHelperFileContent(isGenerateTests));
                    ConversionHelper.unsetUsedCV();
                }
            }
            if (shouldGenerateMainClass()) {
                fileContents.add(getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
            } else if (shouldGenerateSimulatorInterface()) {
                fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
            }
        }

        if (componentInstanceSymbol instanceof LoopComponentInstanceSymbol) {
            EMAEquationSystem equationSystem = ((LoopComponentInstanceSymbol) componentInstanceSymbol).getEquationSystem();
            if (!equationSystemsAlreadyBuild.contains(equationSystem)) {
                equationSystem.setName(Names.getQualifiedName(rootModel.getFullName(),
                        equationSystem.getName()));
                CPPEquationSystemHelper.handleSubComponents(equationSystem);
                fileContents.addAll(generateStrings(taggingResolver,
                        new EquationSystemComponentInstanceSymbol(equationSystem)));
                fileContents.addAll(generateStrings(taggingResolver,
                        new RHSComponentInstanceSymbol(equationSystem)));
            }
        }

        if (!streamTestGenerationMode)
            fileContents.add(new FileContent(generateString(taggingResolver, componentInstanceSymbol), componentInstanceSymbol));
        else
            fileContents.add(new FileContent(generateString(taggingResolver, componentInstanceSymbol),
                    componentInstanceSymbol.getPackageName().replaceAll("\\.", "\\/") + "/" + StringUtils.capitalize(componentInstanceSymbol.getName()) + "Test" + testNamePostFix + ".stream"));

        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            fileContents.add(OctaveHelper.getOctaveHelperFileContent());
        if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend"))
            fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent(isGenerateTests));

        if (componentInstanceSymbol instanceof EMADynamicComponentInstanceSymbol) {
            //TODO: add Events Value Helper
            if (!((EMADynamicComponentInstanceSymbol) componentInstanceSymbol).getEventHandlers().isEmpty())
                fileContents.add(EventPortValueCheck.getEventPortValueCheckFileContent());

            if (((EMADynamicComponentInstanceSymbol) componentInstanceSymbol).isDynamic()) {
                fileContents.add(DynamicHelper.getDynamicHelperFileContent());
            }
        }

        if (shouldGenerateMainClass()) {
            fileContents.add(getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        return fileContents;
    }

    //TODO add incremental generation based on described concept
    public List<File> generateFiles(TaggingResolver taggingResolver, EMAComponentInstanceSymbol componentSymbol) throws IOException {
        this.rootModel = componentSymbol;
        
        List<FileContent> fileContents = new ArrayList<>();
        if (componentSymbol == null) {
            ComponentScanner componentScanner = new ComponentScanner(getModelsDirPath(), taggingResolver, "emam");
            Set<String> availableComponents = componentScanner.scan();
            for (String componentFullName : availableComponents) {
                componentFullName = NameHelper.toInstanceFullQualifiedName(componentFullName);
                if (taggingResolver.resolve(componentFullName,
                        EMAComponentInstanceSymbol.KIND).isPresent()) {
                    EMAComponentInstanceSymbol componentInstanceSymbol = (EMAComponentInstanceSymbol) taggingResolver.resolve(componentFullName,
                            EMAComponentInstanceSymbol.KIND).get();

                    // TODO add properties
                    ExecutionSemantics semantics = new ExecutionSemantics(taggingResolver, componentInstanceSymbol);
                    semantics.addExecutionSemantics();
                    fileContents.addAll(generateStrings(taggingResolver, componentInstanceSymbol));
                }
            }
        } else {
            ExecutionSemantics semantics = new ExecutionSemantics(taggingResolver, rootModel);
            semantics.addExecutionSemantics();

            searchForCVEverywhere(componentSymbol, taggingResolver);
            fileContents = generateStrings(taggingResolver, componentSymbol);
        }
        fileContents.addAll(generateTypes(TypeConverter.getTypeSymbols()));
        fileContents.addAll(handleTestAndCheckDir(taggingResolver, componentSymbol));
        if (isGenerateAutopilotAdapter()) {
            fileContents.addAll(getAutopilotAdapterFiles(componentSymbol));
        }
        if (isGenerateServerWrapper()) {
            fileContents.addAll(getServerWrapperFiles(componentSymbol));
        }
        //System.out.println(fileContents);
        if (getGenerationTargetPath().charAt(getGenerationTargetPath().length() - 1) != '/') {
            setGenerationTargetPath(getGenerationTargetPath() + "/");
        }
        // Add advanceTime
        if (ExecutionStepperHelper.isUsed()) {
            fileContents.add(ExecutionStepperHelper.getTimeHelperFileContent());
        }
        fileContents.addAll(currentFileContentList);
        List<File> files = saveFilesToDisk(fileContents);
        //cmake
        if (generateCMake)
            files.addAll(generateCMakeFiles(componentSymbol));

        return files;
    }

    protected List<File> generateCMakeFiles(EMAComponentInstanceSymbol componentInstanceSymbol) {
        List<File> files = new ArrayList<>();
        if (componentInstanceSymbol != null) {
            cMakeConfig.getCMakeListsViewModel().setCompName(componentInstanceSymbol.getFullName().replace('.', '_').replace('[', '_').replace(']', '_'));
        }
        List<FileContent> contents = cMakeConfig.generateCMakeFiles();
        try {
            for (FileContent content : contents)
                files.add(generateFile(content));
        } catch (IOException e) {
            e.printStackTrace();
        }
        return files;
    }

    public List<File> saveFilesToDisk(List<FileContent> fileContents) throws IOException {
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {
            files.add(generateFile(fileContent));
        }
        return files;
    }

    public List<FileContent> handleTestAndCheckDir(Scope symtab, EMAComponentInstanceSymbol componentSymbol) {
        List<FileContent> fileContents = new ArrayList<>();
        if (isGenerateTests() || isCheckModelDir()) {
            TestsGeneratorCPP g = new TestsGeneratorCPP(this);
            List<FileContent> fileConts = g.generateStreamTests(symtab, componentSymbol);
            fileContents.addAll(fileConts);
        }
        return fileContents;
    }

    public File generateFile(FileContent fileContent) throws IOException {
        File f = new File(getGenerationTargetPath() + fileContent.getFileName());
        Log.info(f.getName(), "FileCreation:");
        boolean contentEqual = false;
        //Actually slower than just saving and overwriting the file
        /*if (f.exists()) {
            String storedFileContent = new String(Files.readAllBytes(f.toPath()));
            if (storedFileContent.equals(fileContent.getFileContent())) {
                contentEqual = true;
            }
        } else*/
        if (!f.exists()) {
            f.getParentFile().mkdirs();
            if (!f.createNewFile()) {
                Log.error("File could not be created");
            }
        }

        if (!contentEqual) {
            BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(f));
            bufferedWriter.write(fileContent.getFileContent(), 0, fileContent.getFileContent().length());
            bufferedWriter.close();
        }
        return f;
    }

    public FileContent getMainClassFileContent(EMAComponentInstanceSymbol instanceSymbol, FileContent fileContent) {
        FileContent newFileContent = new FileContent();

        newFileContent.setFileName("main.cpp");
        String contentString = "#include \"" + fileContent.getFileName() + "\"\n";

        contentString += "int main(int argc, char **argv)\n" +
                "{\n";
        contentString += fileContent.getFileName().substring(0, fileContent.getFileName().indexOf(".")) + " " + instanceSymbol.getName() + "Instance;\n";
        contentString += instanceSymbol.getName() + "Instance.init();\n";
        contentString += "}";
        newFileContent.setFileContent(contentString);


        return newFileContent;
    }

    @Override
    public boolean useAlgebraicOptimizations() {
        return algebraicOptimizations;
    }

    @Override
    public void setUseAlgebraicOptimizations(boolean useAlgebraicOptimizations) {
        this.algebraicOptimizations = useAlgebraicOptimizations;
    }

    @Override
    public boolean useThreadingOptimizations() {
        return threadingOptimizations;
    }

    @Override
    public void setUseThreadingOptimization(boolean useThreadingOptimizations) {
        this.threadingOptimizations = useThreadingOptimizations;
    }

    @Override
    public MathCommandRegister getMathCommandRegister() {
        return mathCommandRegister;
    }

    @Override
    public void setMathCommandRegister(MathCommandRegister mathCommandRegister) {
        this.mathCommandRegister = mathCommandRegister;
    }

    public boolean shouldGenerateMainClass() {
        return generateMainClass;
    }

    public void setGenerateMainClass(boolean generateMainClass) {
        this.generateMainClass = generateMainClass;
    }

    public boolean shouldGenerateSimulatorInterface() {
        return generateSimulatorInterface;
    }

    public void setGenerateSimulatorInterface(boolean generateSimulatorInterface) {
        this.generateSimulatorInterface = generateSimulatorInterface;
    }

    public boolean useMPIDefinitionFix() {
        return MPIDefinitionFix;
    }

    public void setUseMPIDefinitionFix(boolean useFix) {
        this.MPIDefinitionFix = useFix;
    }

    public Path getModelsDirPath() {
        return modelsDirPath;
    }

    public void setModelsDirPath(Path modelsDirPath) {
        this.modelsDirPath = modelsDirPath;
    }

    public boolean isGenerateTests() {
        return isGenerateTests;
    }

    public void setGenerateTests(boolean generateTests) {
        isGenerateTests = generateTests;
    }

    public boolean isGenerateAutopilotAdapter() {
        return isGenerateAutopilotAdapter;
    }

    public void setGenerateAutopilotAdapter(boolean generateAutopilotAdapter) {
        isGenerateAutopilotAdapter = generateAutopilotAdapter;
    }

    public boolean isGenerateServerWrapper() {
        return isGenerateServerWrapper;
    }

    public void setGenerateServerWrapper(boolean generateServerWrapper) {
        isGenerateServerWrapper = generateServerWrapper;
    }

    public boolean isCheckModelDir() {
        return checkModelDir;
    }

    public void setCheckModelDir(boolean checkModelDir) {
        this.checkModelDir = checkModelDir;
    }

    public List<EMAMBluePrintCPP> getBluePrints() {
        return Collections.unmodifiableList(bluePrints);
    }

    private static List<FileContent> generateTypes(Collection<MCTypeSymbol> typeSymbols) {
        TypesGeneratorCPP tg = new TypesGeneratorCPP();
        return tg.generateTypes(typeSymbols);
    }

    private static List<FileContent> getAutopilotAdapterFiles(EMAComponentInstanceSymbol componentSymbol) {
        List<FileContent> result = new ArrayList<>();

        AutopilotAdapterDataModel dm = new AutopilotAdapterDataModel();
        dm.setMainModelName(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        dm.setInputCount(componentSymbol.getIncomingPortInstances().size());
        dm.setOutputCount(componentSymbol.getOutgoingPortInstances().size());
        for (EMAPortInstanceSymbol port : componentSymbol.getIncomingPortInstances()) {
            dm.addInput(port.getName(), port.getTypeReference().getName());
        }
        for (EMAPortInstanceSymbol port : componentSymbol.getOutgoingPortInstances()) {
            dm.addOutput(port.getName(), port.getTypeReference().getName());
        }

        result.add(generateAutopilotAdapterH(dm));
        result.add(generateAutopilotAdapterCpp(dm));
        return result;
    }

    private static FileContent generateAutopilotAdapterH(AutopilotAdapterDataModel dm) {
        String fileContents = AllTemplates.generateAutopilotAdapterH(dm);
        return new FileContent(fileContents, "AutopilotAdapter.h");
    }

    private static FileContent generateAutopilotAdapterCpp(AutopilotAdapterDataModel dm) {
        String fileContents = AllTemplates.generateAutopilotAdapterCpp(dm);
        if (currentInstance.generateCMake)
            addAutopilotAdapterCMakeConfig();
        return new FileContent(fileContents, "AutopilotAdapter.cpp");
    }

    private static void addAutopilotAdapterCMakeConfig() {
        CMakeConfig cmake = currentInstance.cMakeConfig;
        // add jni
        cmake.addCMakeCommand("find_package(JNI)");
        cmake.addCMakeCommand("set(INCLUDE_DIRS ${INCLUDE_DIRS} ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})");
        // create shared lib
        cmake.addCMakeCommandEnd("add_library(AutopilotAdapter SHARED AutopilotAdapter.cpp ${CMAKE_CURRENT_SOURCE_DIR})");
        cmake.addCMakeCommandEnd("target_include_directories(AutopilotAdapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})");
        cmake.addCMakeCommandEnd("target_link_libraries(AutopilotAdapter PUBLIC ${LIBS})");
        cmake.addCMakeCommandEnd("set_target_properties(AutopilotAdapter PROPERTIES LINKER_LANGUAGE CXX)");
        cmake.addCMakeCommand("IF (WIN32)");
        cmake.addCMakeCommandEnd("set_target_properties(AutopilotAdapter PROPERTIES PREFIX \"\")");
        cmake.addCMakeCommand("ENDIF()");
        // install shared lib
        cmake.addCMakeCommandEnd("install(TARGETS AutopilotAdapter DESTINATION $ENV{DLL_DIR})");
        cmake.addCMakeCommandEnd("export(TARGETS AutopilotAdapter FILE de_rwth_armin_modeling_autopilot_autopilotAdapter.cmake)");
    }

    private static List<FileContent> getServerWrapperFiles(EMAComponentInstanceSymbol componentSymbol) {
        List<FileContent> result = new ArrayList<>();
        String[] filesToCopy = new String[]{
                "Makefile",
                "model.proto"
        };
        for (String file : filesToCopy) {
            String resourcePath = String.format("/template/serverwrapper/%s", file);
            result.add(FileUtil.getResourceAsFile(resourcePath, file));
        }
        result.add(generateServerWrapper(componentSymbol));
        return result;
    }

    private static FileContent generateServerWrapper(EMAComponentInstanceSymbol componentSymbol) {
        return generateWrapper(componentSymbol, "server.cc");
    }

    private static FileContent generateWrapper(EMAComponentInstanceSymbol componentSymbol, String name) {
        ServerWrapperViewModel vm = new ServerWrapperViewModel();
        vm.setMainModelName(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        String fileContents = AllTemplates.generateServerWrapper(vm);
        return new FileContent(fileContents, name);
    }

    public boolean isGenerateCMakeEnabled() {
        return generateCMake;
    }

    public void setGenerateCMake(boolean generateCMake) {
        if (!this.generateCMake && generateCMake)
            setupCMake();
        this.generateCMake = generateCMake;
    }

    public CMakeConfig getCMakeConfig() {
        return cMakeConfig;
    }


    public MathOptSolverConfig getMathOptSolverConfig() {
        return mathOptSolverConfig;
    }

    public OptimizationSymbolHandler getMathOptExecuteMethodGenerator() {
        return mathOptExecuteMethodGenerator;
    }

    public void searchForCVEverywhere(EMAComponentInstanceSymbol componentInstanceSymbol, Scope symtab) {
        MathStatementsSymbol mathStatementsSymbol = Helper.getMathStatementsSymbolFor(componentInstanceSymbol, symtab);
        if (mathStatementsSymbol != null) {
            List<MathExpressionSymbol> mathExpressionSymbols = mathStatementsSymbol.getMathExpressionSymbols();
            for (MathExpressionSymbol mathExpressionSymbol : mathExpressionSymbols) {
                String nameOfFunction = ComponentConverter.getNameOfMathCommand(mathExpressionSymbol);
                MathCommand mathCommand = this.mathCommandRegister.getMathCommand(nameOfFunction);
                if (mathCommand != null) {
                    if (mathCommand.isCVMathCommand()) {
                        this.isGenerateCV = true;
                    }
                }

            }
        }
        for (EMAComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {
            searchForCVEverywhere(instanceSymbol, symtab);
        }
    }

    public double getDeltaT() {
        return ExecutionStepperHelper.getDT();
    }

    public void setDeltaT(double dt) {
        ExecutionStepperHelper.setDT(dt);
    }

    public void setDeltaT(String dt) {
        double v = Double.parseDouble(dt);
        setDeltaT(v);
    }
}
