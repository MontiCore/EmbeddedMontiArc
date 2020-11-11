/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.FileContent;

import java.io.InputStream;
import java.util.Scanner;

public final class FileUtil {

    private FileUtil() {
    }

    public static FileContent getResourceAsFile(String resourcePath, String destinationFilePath) {
        InputStream resource = FileUtil.class.getResourceAsStream(resourcePath);
        if (resource == null) throw new IllegalArgumentException("Could not get resource: "+resourcePath);
        String body = new Scanner(resource, "UTF-8").useDelimiter("\\A").next();
        return new FileContent(body, destinationFilePath);
    }
}
