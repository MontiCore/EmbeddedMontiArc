/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

public class ExecutionStepperHelper {

    private static double DT = 0.1;

    public static String FILENAME = "ExecutionStepper";

    public static String CURRENTTIME_METHOD_NAME = "getCurrentTime";

    private static boolean TIME_USED = false;

    public static void setUsed() {
        TIME_USED = true;
    }

    public static void setUnused() {
        TIME_USED = false;
    }

    public static boolean isUsed() {
        return TIME_USED;
    }

    public static FileContent getTimeHelperFileContent(double dt) {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(FILENAME + ".h");
        String fileContentString = getTimeHelperSourceCode(dt);

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

    public static FileContent getTimeHelperFileContent(){
        return getTimeHelperFileContent(DT);
    }

    public static String getTimeHelperSourceCode(double dt) {
        return "#ifndef EXECUTIONSTEPPER\n" +
                "#define EXECUTIONSTEPPER\n" +
                "\n" +
                String.format("static double static_var_dt = %s;\n", dt) +
                String.format("static double static_var_currentTime = %s;\n", dt) +
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
        return de.monticore.lang.monticar.semantics.Constants.timeName;
    }

    public static double getDT() {
        return DT;
    }

    public static void setDT(double dt) {
        ExecutionStepperHelper.DT = DT;
    }
}
