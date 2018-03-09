package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;

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
}
