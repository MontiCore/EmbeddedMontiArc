package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.Variable;

public class PrinterHelper {
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

    public static String printClass(BluePrint bluePrint) {
        StringBuilder builder = new StringBuilder();
        builder.append("class " + bluePrint.getName() + "{\n");

        bluePrint.getVariables().forEach(v -> builder.append("\t" + PrinterHelper.printVariable(v) + "\n"));
        builder.append("\n");
        //TODO: give each method own access modifier
        builder.append("public:\n");
        bluePrint.getMethods().forEach(m -> builder.append(PrinterHelper.printMethod(m) + "\n"));
        builder.append("};");

        return builder.toString();
    }
}
