/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.util;

import java.io.File;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Objects;

/**
 *
 */
public class FileUtils {
    public static boolean deleteDirectory(File directory) {
        if (directory.exists()) {
            File[] files = directory.listFiles();
            assert files != null;
            for (File f : files) {
                if (f.isDirectory()) {
                    deleteDirectory(f);
                } else {
                    f.delete();
                }
            }
        }
        return directory.delete();
    }

    public static boolean deleteDirectory(String pathToDirectory) {
        File file = new File(pathToDirectory);
        return deleteDirectory(file);
    }

    public static File loadFileFromJavaContext(final String resourcePath) {
        try {
            URI uriToFile = Objects.requireNonNull(FileUtils.class.getClassLoader().getResource(resourcePath)).toURI();
            return new File(uriToFile);
        } catch (URISyntaxException e) {
            throw new IllegalArgumentException("Resource path is cannot be loaded as URI:" + e.getMessage());
        }
    }
}
