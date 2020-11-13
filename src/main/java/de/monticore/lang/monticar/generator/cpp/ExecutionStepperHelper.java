/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

public class ExecutionStepperHelper {
    public static String fileName = "ExecutionStepper";

    private static boolean usedTime = false;

    public static void setUsed() {
        usedTime = true;
    }

    public static boolean isUsed() {
        return usedTime;
    }

    public static FileContent getTimeHelperFileContent(double dt) {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = getTimeHelperSourceCode(dt);

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

    public static FileContent getTimeHelperFileContent(){
        return getTimeHelperFileContent(1);
    }

    public static String getTimeHelperSourceCode(double dt) {
        return "#ifndef EXECUTIONSTEPPER\n" +
                "#define EXECUTIONSTEPPER\n" +
                "\n" +
                "static double static_var_dt = " + dt + ";\n" +
                "static double static_var_currentTime = 0;\n" +
                "\n" +
                "static double getCurrentTime() {\n" +
                "    return static_var_currentTime;\n" +
                "}\n" +
                "static void advanceTime() {\n" +
                "    static_var_currentTime += static_var_dt;\n" +
                "}\n" +
                "#endif\n";
    }

    public static String getTimeVariableName() {
        return de.monticore.lang.monticar.semantics.Options.timeName;
    }
}
