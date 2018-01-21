package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

//TODO arrays
//TODO:implements Generator
public class GeneratorRosCpp {

    private String generationTargetPath;

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

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath(generationTargetPath);
        generatorCPP.useArmadilloBackend();
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
        builder.append("#include \"MsgPortHelper.h\"\n");
        builder.append("#include \"" + componentSymbol.getFullName().replace(".", "_") + ".h\"\n");

        Set importStringSet = new HashSet();
        DataHelper.getTopics().forEach(t -> importStringSet.add("<" + t.getImportString() + ".h>"));
        importStringSet.forEach(i -> builder.append("#include " + i + "\n"));

        String classname = currentBluePrint.getName();

        res.setFileName(classname + ".h");
        //class
        builder.append("class " + classname + "{\n");

        currentBluePrint.getVariables().forEach(v -> builder.append("\t" + printVariable(v) + "\n"));
        builder.append("\n");
        //TODO: give each method own access modifier
        builder.append("public:\n");
        currentBluePrint.getMethods().forEach(m -> builder.append(printMethod(m) + "\n"));
        builder.append("};");

        res.setFileContent(builder.toString());
        return res;
    }

    private String printVariable(Variable v) {
        return v.getVariableType().getTypeNameTargetLanguage() + " " +
                v.getNameTargetLanguageFormat() + ";";
    }

    private String printMethod(Method method) {
        String res = method.getReturnTypeName() + " " + method.getName() + "(";
        res = res.trim();
        res = "\t" + res;
        for (Variable param : method.getParameters()) {
            res += param.getVariableType().getTypeNameTargetLanguage() + " " + param.getNameTargetLanguageFormat();
            res += ", ";
        }
        //remove last ", "
        if (method.getParameters().size() > 0)
            res = res.substring(0, res.length() - 2);
        res += ")";
        if (method.getInstructions().size() == 0) {
            res += ";\n";
        } else {
            res += "{\n";
            for (Instruction instr : method.getInstructions()) {
                res += "\t\t" + instr.getTargetLanguageInstruction() + "\n";
            }
            res += "\t}\n";
        }
        return res;
    }
}
