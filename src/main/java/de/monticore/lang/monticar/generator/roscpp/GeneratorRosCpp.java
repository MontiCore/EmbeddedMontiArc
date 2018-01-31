package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.roscpp.helper.PrinterHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class GeneratorRosCpp {

    private String generationTargetPath;
    private boolean generateCpp = true;
    private GeneratorCPP generatorCPP;

    //TODO: as tag?
    private ExecutionStrategy executionStrategy;

    public void setExecutionStrategy(ExecutionStrategy executionStrategy) {
        this.executionStrategy = executionStrategy;
    }

    public void setGenerateCpp(boolean generateCpp) {
        this.generateCpp = generateCpp;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public List<File> generateFiles(ResolvedRosTag resolvedRosTag, TaggingResolver symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(symtab, resolvedRosTag);

        if (getGenerationTargetPath().charAt(getGenerationTargetPath().length() - 1) != '/') {
            setGenerationTargetPath(getGenerationTargetPath() + "/");
        }
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {

            files.add(generateFile(fileContent));
        }

        if (generateCpp) {
            files.addAll(generateCppFiles(symtab, resolvedRosTag.getComponent()));
        }

        return files;
    }

    public File generateFile(FileContent fileContent) throws IOException {
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

    public List<FileContent> generateStrings(Scope scope, ResolvedRosTag resolvedRosTag) {
        List<FileContent> fileContents = new ArrayList<>();
        fileContents.add(generateRosCompUnit(resolvedRosTag));
        return fileContents;
    }

    public void setGeneratorCPP(GeneratorCPP generatorCPP) {
        this.generatorCPP = generatorCPP;
    }

    private List<File> generateCppFiles(TaggingResolver taggingResolver, ExpandedComponentInstanceSymbol symbol) throws IOException {
        //If user does not specify otherwise init with useful defaults
        if (generatorCPP == null) {
            generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
        }
        //TODO: let user specify 2 different paths?
        generatorCPP.setGenerationTargetPath(generationTargetPath);
        return generatorCPP.generateFiles(symbol, taggingResolver);
    }

    private FileContent generateRosCompUnit(ResolvedRosTag resolvedRosTag) {
        FileContent res = new FileContent();

        LanguageUnitRosCppWrapper languageUnitRosCppWrapper = new LanguageUnitRosCppWrapper();
        languageUnitRosCppWrapper.generateBluePrints(resolvedRosTag);
        //TODO: unsave, does not work with multiple
        BluePrintCPP currentBluePrint = languageUnitRosCppWrapper.getBluePrints().get(0);
        //TODO pull down into LanguageUnitRosCppWrapper
        if (executionStrategy != null)
            executionStrategy.generate(currentBluePrint);

        String classname = currentBluePrint.getName();
        res.setFileName(classname + ".h");

        res.setFileContent(PrinterHelper.printClass(currentBluePrint));
        return res;
    }

}
