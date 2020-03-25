/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

/**
 */
public class ArmadilloHelper {
    public static String fileName = "HelperA";

    public static FileContent getArmadilloHelperFileContent(boolean generateTests) {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = ArmadilloHelperSource.armadilloHelperSourceCode;

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

    public static FileContent getArmadilloHelperFileContent(){
        return getArmadilloHelperFileContent(false);
    }
}
