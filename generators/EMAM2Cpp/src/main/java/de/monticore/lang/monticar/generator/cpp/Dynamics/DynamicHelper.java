/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.Dynamics;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;

public class DynamicHelper {

    public static String fileName = "DynamicHelper";

    public static FileContent getDynamicHelperFileContent() {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = AllTemplates.generateDynamicHelper();

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }

}
