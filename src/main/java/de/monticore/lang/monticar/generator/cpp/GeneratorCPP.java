package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.Generator;
import de.monticore.lang.monticar.generator.Helper;
import de.monticore.lang.monticar.generator.MathCommandRegister;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.monticar.generator.cpp.resolver.SymTabCreator;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class GeneratorCPP implements Generator {
    protected String generationTargetPath = "./target/generated-sources-cpp/";

    protected boolean algebraicOptimizations = false;
    protected boolean threadingOptimizations = false;
    protected boolean MPIDefinitionFix = true;
    protected MathCommandRegister mathCommandRegister;
    protected boolean generateMainClass = false;
    protected boolean generateSimulatorInterface = false;

    public GeneratorCPP() {
        this.mathCommandRegister = new MathCommandRegisterCPP();
    }

    public static void main(String[] args) throws IOException, URISyntaxException {
        Path resolvingPath = Paths.get(args[0]);
        String fullName = args[1];
        String outputPath = args[2];

        SymTabCreator symTabCreator = new SymTabCreator(resolvingPath);
        Scope symtab = symTabCreator.createSymTab();
        Resolver resolver = new Resolver(symtab);

        ExpandedComponentInstanceSymbol componentSymbol = resolver.getExpandedComponentInstanceSymbol(fullName)
                .orElseThrow(() -> new IllegalArgumentException("Argument must include the full component name"));

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath(outputPath);
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    public String generateString(ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab) {
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentInstanceSymbol, symtab);

        return generateString(componentInstanceSymbol, mathSymbol);
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
    public String generateString(ExpandedComponentInstanceSymbol componentSymbol, MathStatementsSymbol mathStatementsSymbol) {
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

        String result = languageUnitCPP.getGeneratedHeader(bluePrintCPP);
        return result;
    }

    public static List<FileContent> currentFileContentList = null;

    @Override
    public List<FileContent> generateStrings(ExpandedComponentInstanceSymbol componentInstanceSymbol, Scope symtab) {
        List<FileContent> fileContents = new ArrayList<>();
        if (componentInstanceSymbol.getFullName().equals("simulator.mainController")) {
            setGenerateSimulatorInterface(true);
        } else {
            //setGenerateMainClass(true);
        }
        currentFileContentList = fileContents;
        fileContents.add(new FileContent(generateString(componentInstanceSymbol, symtab), componentInstanceSymbol));
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

                fileContents.addAll(generateStrings(instanceSymbol, symtab));
            }
        }

        fileContents.add(OctaveHelper.getOctaveHelperFileContent());

        if (shouldGenerateMainClass()) {
            //fileContents.add(getMainClassFileContent(componentInstanceSymbol, fileContents.get(0)));
        } else if (shouldGenerateSimulatorInterface()) {
            fileContents.addAll(SimulatorIntegrationHelper.getSimulatorIntegrationHelperFileContent());
        }

        return fileContents;
    }

    public List<File> generateFiles(ExpandedComponentInstanceSymbol componentSymbol, Scope symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(componentSymbol, symtab);
        //System.out.println(fileContents);
        if (getGenerationTargetPath().charAt(getGenerationTargetPath().length() - 1) != '/') {
            setGenerationTargetPath(getGenerationTargetPath() + "/");
        }
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {

            files.add(generateFile(fileContent));
        }
        return files;
    }

    public File generateFile(FileContent fileContent) throws IOException{
        File f = new File(getGenerationTargetPath() + fileContent.getFileName());
        Log.info(f.getName(), "FileCreation:");
        if (!f.exists()) {
            f.getParentFile().mkdirs();
            if (!f.createNewFile()) {
                Log.error("File could not be created");
            }
        }
        BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(f));
        bufferedWriter.write(fileContent.getFileContent(), 0, fileContent.getFileContent().length());
        bufferedWriter.close();
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
}
