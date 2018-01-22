package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

//TODO arrays
//TODO:implements Generator
public class GeneratorRosCpp {

    private String generationTargetPath;
    private boolean generateCpp = true;
    private GeneratorCPP generatorCPP;

    public void setGenerateCpp(boolean generateCpp) {
        this.generateCpp = generateCpp;
    }

    public String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    public List<File> generateFiles(ExpandedComponentInstanceSymbol componentSymbol, TaggingResolver symtab) throws IOException {
        List<FileContent> fileContents = generateStrings(symtab, componentSymbol);

        if (getGenerationTargetPath().charAt(getGenerationTargetPath().length() - 1) != '/') {
            setGenerationTargetPath(getGenerationTargetPath() + "/");
        }
        List<File> files = new ArrayList<>();
        for (FileContent fileContent : fileContents) {

            files.add(generateFile(fileContent));
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

    public List<FileContent> generateStrings(Scope scope, ExpandedComponentInstanceSymbol symbol) {
        List<FileContent> fileContents = new ArrayList<>();

        fileContents.add(generateRosCompUnit(symbol));

        if (generateCpp) {
            fileContents.addAll(generateCppStrings(scope, symbol));
        }

        return fileContents;
    }

    public void setGeneratorCPP(GeneratorCPP generatorCPP) {
        this.generatorCPP = generatorCPP;
    }

    private List<FileContent> generateCppStrings(Scope scope, ExpandedComponentInstanceSymbol symbol) {
        //If user does not specify otherwise init with useful defaults
        if (generatorCPP == null) {
            generatorCPP = new GeneratorCPP();
            generatorCPP.useArmadilloBackend();
        }
        //TODO: let user specify 2 different paths?
        generatorCPP.setGenerationTargetPath(generationTargetPath);
        List<FileContent> fileContents = new ArrayList<>();
        fileContents.addAll(generatorCPP.generateStrings((TaggingResolver) scope, symbol, scope));
        //TODO: dirty fix of import
        fileContents.forEach(fc -> fc.setFileContent(fc.getFileContent().replace("armadillo.h", "armadillo")));

        return fileContents;
    }


    private FileContent generateRosCompUnit(ExpandedComponentInstanceSymbol componentSymbol) {
        FileContent res = new FileContent();

        LanguageUnitRosCppWrapper languageUnitRosCppWrapper = new LanguageUnitRosCppWrapper();
        languageUnitRosCppWrapper.addSymbolToConvert(componentSymbol);
        languageUnitRosCppWrapper.generateBluePrints();
        //TODO: unsave, does not work with multiple
        BluePrint currentBluePrint = languageUnitRosCppWrapper.getBluePrints().get(0);

        //imports
        StringBuilder builder = new StringBuilder();
        //TODO: add to blueprint
        builder.append("#pragma once\n");
        builder.append("#include <ros/ros.h>\n");
        builder.append("#include \"" + componentSymbol.getFullName().replace(".", "_") + ".h\"\n");

        //Add each import exactly once
        DataHelper.getTopics().stream()
                .map(t -> "#include <" + t.getImportString() + ".h>\n")
                .distinct()
                .forEach(builder::append);

        String classname = currentBluePrint.getName();

        res.setFileName(classname + ".h");
        //class
        builder.append(PrinterHelper.printClass(currentBluePrint));

        res.setFileContent(builder.toString());
        return res;
    }

}
