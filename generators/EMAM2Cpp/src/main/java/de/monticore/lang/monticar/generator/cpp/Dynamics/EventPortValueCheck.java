/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.Dynamics;

import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.ArmadilloHelperSource;
import de.monticore.lang.monticar.generator.cpp.template.AllTemplates;
import freemarker.template.Template;

/**
 *
 */
public class EventPortValueCheck {
    public static String fileName = "PortValueCheck";

    public static FileContent getEventPortValueCheckFileContent() {
        FileContent fileContent = new FileContent();
        fileContent.setFileName(fileName + ".h");
        String fileContentString = AllTemplates.generateDynamicEventsPortValueCheck();

        fileContent.setFileContent(fileContentString);
        return fileContent;
    }
}
