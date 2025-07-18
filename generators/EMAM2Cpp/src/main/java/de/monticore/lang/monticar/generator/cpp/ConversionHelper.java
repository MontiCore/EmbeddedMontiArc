/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

/**
 * @author Ahmed Diab
 */
public class ConversionHelper {
    public static String fileName = "ConvHelper";
    private static boolean usedCV = false;

    public static FileContent getConversionHelperFileContent(boolean generateTests) {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = ConversionHelperSource.conversionHelperSourceCode;

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

    public static FileContent getConversionHelperFileContent(){
        return getConversionHelperFileContent(false);
    }

    public static boolean isUsedCV() {
        return usedCV;
    }

    public static void setUsedCV() {
        usedCV = true;
    }

    public static void unsetUsedCV() {
        usedCV = false;
    }
}
