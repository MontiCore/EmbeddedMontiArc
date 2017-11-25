package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

/**
 * @author Sascha Schneiders
 */
public class ArmadilloHelper {
    public static String fileName = "HelperA";

    public static FileContent getArmadilloHelperFileContent() {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = ArmadilloHelperSource.armadilloHelperSourceCode;

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }
}
