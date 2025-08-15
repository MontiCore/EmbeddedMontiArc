/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.monticar.generator.rosmsg.util.FileContent;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class PrinterHelper {

    private PrinterHelper() {
    }

    public static File generateFile(FileContent fileContent, String targetPath) throws IOException {
        File f = new File(targetPath + fileContent.getFileName());
        Log.info(f.getName(), "FileCreation:");
        if (!f.exists()) {
            f.getParentFile().mkdirs();
            if (!f.createNewFile()) {
                Log.error("File could not be created");
            }
        }
        BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(f));
        bufferedWriter.write(fileContent.getFileContent(), 0, fileContent.getFileContent().length());
        bufferedWriter.close();
        return f;
    }
}
