/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.mqtt.helper;

import java.io.File;

public class FilesHelper {

    private File file;
    private String content;

    public FilesHelper(File file, String content) {
      this.file = file;
      this.content = content;
    }

    public File getFile(){
      return file;
    }

    public String getContent(){
      return content;
    }
}
