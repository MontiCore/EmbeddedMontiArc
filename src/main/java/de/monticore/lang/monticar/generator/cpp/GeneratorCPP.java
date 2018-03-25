package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.Helper;
import de.monticore.lang.monticar.generator.MathCommandRegister;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.generator.cpp.converter.TypeConverter;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import de.monticore.lang.monticar.generator.cpp.viewmodel.AutopilotAdapterViewModel;
import de.monticore.lang.monticar.generator.cpp.viewmodel.ServerWrapperViewModel;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class GeneratorCPP implements Generator {
    public static GeneratorCPP currentInstance;
    private Path modelsDirPath;
    private boolean isGenerateTests = false;
    private boolean isGenerateAutopilotAdapter = false;
    private boolean isGenerateServerWrapper = false;
    private final List<BluePrintCPP> bluePrints = new ArrayList<>();

    protected String generationTargetPath = "./target/generated-sources-cpp/";

    protected boolean algebraicOptimizations = false;
    protected boolean threadingOptimizations = false;
    protected boolean MPIDefinitionFix = true;
    protected MathCommandRegister mathCommandRegister;
    protected boolean generateMainClass = false;
    protected boolean generateSimulatorInterface = false;
    protected boolean checkModelDir = false;

    public GeneratorCPP() {
        this.mathCommandRegister = new MathCommandRegisterCPP();
        useOctaveBackend();
        TypeConverter.clearTypeSymbols();
        currentInstance = this;
    }


    public void useArmadilloBackend() {
        MathConverter.curBackend = new ArmadilloBackend();
    }

    public void useOctaveBackend() {
        MathConverter.curBackend = new OctaveBackend();
        //Log.warn("This backend has been deprecated. Armadillo is the recommended backend now.");
    }

    public String generateString(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab) {
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentInstanceSymbol, symtab);

        return generateString(taggingResolver, componentInstanceSymbol, mathSymbol);
    }

    @Override
    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    @Override
    public void setGenerationTargetPath(String newPath) {
        this.generationTargetPath = newPath;
    }

    @Override
    public String generateString(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol) {
        LanguageUnitCPP languageUnitCPP = new LanguageUnitCPP();
        languageUnitCPP.setGeneratorCPP(this);
        languageUnitCPP.addSymbolToConvert(componentSymbol);
        if (mathStatementsSymbol != null)
            languageUnitCPP.addSymbolToConvert(mathStatementsSymbol);
        languageUnitCPP.generateBluePrints();

        BluePrintCPP bluePrintCPP = null;
        for (BluePrint bluePrint : languageUnitCPP.getBluePrints()) {
            if (bluePrint.getOriginalSymbol().equals(componentSymbol)) {
                bluePrintCPP = (BluePrintCPP) bluePrint;
            }
        }

        if (bluePrintCPP != null) {
            bluePrints.add(bluePrintCPP);
        }

        String result = languageUnitCPP.getGeneratedHeader(taggingResolver, bluePrintCPP);
        return result;
    }

    public static List<FileContent> currentFileContentList = null;

    @Override
    public List<FileContent> generateStrings(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab) {
        List<FileContent> fileContents = new ArrayList<>();
        if (componentInstanceSymbol.getFullName().equals("simulator.mainController")) {
            setGenerateSimulatorInterface(true);
        } else {
            //setGenerateMainClass(true);
        }
        currentFileContentList = fileContents;
        fileContents.add(new FileContent(generateString(taggingResolver, componentInstanceSymbol, symtab), componentInstanceSymbol));
        String lastNameWithoutArrayPart = "";
        for (ExpandedComponentInstanceSymbol instanceSymbol : componentInstanceSymbol.getSubComponents()) {
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

                fileContents.addAll(generateStrings(taggingResolver, instanceSymbol, symtab));
            }
        }
        if (MathConverter.curBackend.getBackendName().equals("OctaveBackend"))
            fileContents.add(OctaveHelper.getOctaveHelperFileContent());
        if (MathConverter.curBackend.getBackendName().equals("ArmadilloBackend"))
            fileContents.add(ArmadilloHelper.getArmadilloHelperFileContent());

        if (shouldGenerateMainClass()) {
            //fileContents.add(getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        return fileContents;
    }

    //TODO add incremental generation based on described concept
    public List<File> generateFiles(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol componentSymbol,
                                    Scope symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(taggingResolver, componentSymbol, symtab);
        fileContents.addAll(generateTypes(TypeConverter.getTypeSymbols()));
        fileContents.addAll(handleTestAndCheckDir(symtab));
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
        List<File> files = saveFilesToDisk(fileContents);

        return files;
    }

    public List<File> saveFilesToDisk(List<FileContent> fileContents) throws IOException {
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {
            files.add(generateFile(fileContent));
        }
        return files;
    }

    public List<FileContent> handleTestAndCheckDir(Scope symtab) {
        List<FileContent> fileContents = new ArrayList<>();
        if (isGenerateTests() || isCheckModelDir()) {
            TestsGeneratorCPP g = new TestsGeneratorCPP(this);
            List<FileContent> fileConts = g.generateStreamTests(symtab);
            fileContents.addAll(fileConts);
        }
        return fileContents;
    }

    public List<File> generateFiles(ExpandedComponentInstanceSymbol componentSymbol,
                                    TaggingResolver taggingResolver) throws IOException {
        return generateFiles(taggingResolver, componentSymbol, taggingResolver);
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

    public FileContent getMainClassFileContent(ExpandedComponentInstanceSymbol instanceSymbol, FileContent fileContent) {
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

    public List<BluePrintCPP> getBluePrints() {
        return Collections.unmodifiableList(bluePrints);
    }

    private static List<FileContent> generateTypes(Collection<MCTypeSymbol> typeSymbols) {
        TypesGeneratorCPP tg = new TypesGeneratorCPP();
        return tg.generateTypes(typeSymbols);
    }

    private static List<FileContent> getAutopilotAdapterFiles(ExpandedComponentInstanceSymbol componentSymbol) {
        List<FileContent> result = new ArrayList<>();
        result.add(FileUtil.getResourceAsFile("/template/autopilotadapter/AutopilotAdapter.h", "AutopilotAdapter.h"));
        result.add(generateAutopilotAdapter(componentSymbol));
        return result;
    }

    private static FileContent generateAutopilotAdapter(ExpandedComponentInstanceSymbol componentSymbol) {
        AutopilotAdapterViewModel vm = new AutopilotAdapterViewModel();
        vm.setMainModelName(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        String fileContents = AllTemplates.generateAutopilotAdapter(vm);
        return new FileContent(fileContents, "AutopilotAdapter.cpp");
    }

    private static List<FileContent> getServerWrapperFiles(ExpandedComponentInstanceSymbol componentSymbol) {
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

    private static FileContent generateServerWrapper(ExpandedComponentInstanceSymbol componentSymbol) {
        ServerWrapperViewModel vm = new ServerWrapperViewModel();
        vm.setMainModelName(GeneralHelperMethods.getTargetLanguageComponentName(componentSymbol.getFullName()));
        String fileContents = AllTemplates.generateServerWrapper(vm);
        return new FileContent(fileContents, "server.cc");
    }
}
