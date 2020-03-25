/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;

/**
 */
public class FileContent {
    String fileContent;
    String fileName;

    public FileContent() {

    }

    public FileContent(String fileContent, EMAComponentInstanceSymbol instanceSymbol) {
        this.fileContent = fileContent;
        fileName = GeneralHelperMethods.getTargetLanguageComponentName(instanceSymbol.getFullName())+".h";
    }

    public FileContent(String fileContent, String fileName) {
        this.fileContent = fileContent;
        this.fileName = fileName;
    }

    public String getFileContent() {
        return fileContent;
    }

    public void setFileContent(String fileContent) {
        this.fileContent = fileContent;
    }

    public String getFileName() {
        return fileName;
    }

    public void setFileName(String fileName) {
        this.fileName = fileName;
    }
}
