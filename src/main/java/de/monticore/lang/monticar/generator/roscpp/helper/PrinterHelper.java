package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.monticar.generator.roscpp.util.BluePrintCPP;
import de.monticore.lang.monticar.generator.roscpp.util.Instruction;
import de.monticore.lang.monticar.generator.roscpp.util.Method;
import de.monticore.lang.monticar.generator.roscpp.util.Variable;
import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Comparator;

public class PrinterHelper {

    private PrinterHelper() {
    }

    public static String printVariable(Variable v) {
        return v.getVariableType().getTypeNameTargetLanguage() + " " +
                v.getNameTargetLanguageFormat() + ";";
    }

    public static String printMethod(Method method) {
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
        res += "{\n";
        for (Instruction instr : method.getInstructions()) {
            res += "\t\t" + instr.getTargetLanguageInstruction() + "\n";
        }
        res += "\t}\n";
        return res;
    }

    public static String printClass(BluePrintCPP bluePrint, String extendsString) {
        StringBuilder builder = new StringBuilder();

        builder.append("#pragma once\n");

        bluePrint.getAdditionalIncludeStrings().stream()
                .sorted()
                .forEach(inlc -> builder.append("#include " + inlc + "\n"));

        if (extendsString == null || extendsString.equals("")) {
            builder.append("class " + bluePrint.getName() + "{\n");
        } else {
            builder.append("class " + bluePrint.getName() + extendsString + "{\n");
        }
        bluePrint.getConsts().forEach(builder::append);

        bluePrint.getVariables().forEach(v -> builder.append("\t" + PrinterHelper.printVariable(v) + "\n"));
        builder.append("\n");
        //TODO: give each method own access modifier
        builder.append("public:\n");
        bluePrint.getMethods().stream()
                .sorted(Comparator.comparing(Method::getName))
                .forEach(m -> builder.append(PrinterHelper.printMethod(m) + "\n"));
        builder.append("};");

        return builder.toString();
    }

    public static File generateFile(FileContent fileContent, String targetPath) throws IOException {
        File f = new File(targetPath + fileContent.getFileName());
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
}
