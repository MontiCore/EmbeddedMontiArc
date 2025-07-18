/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util;

import de.se_rwth.commons.logging.Log;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Arrays;
import java.util.List;

public class BasicLibrary {
    public static final String BASIC_LIBRARY_ROOT = "target/basiclibrary";

    public static void extract() {
        String basePath = "de.monticore.lang.monticar.semantics.library";
        List<String> arithmetic = Arrays.asList("Add", "Constant", "Division", "Gain", "Max", "Min", "Mod",
                "Multiplication", "Saturation", "Subtract");
        List<String> basicLibrary = Arrays.asList("Delay", "LookUp");
        List<String> differential = Arrays.asList("Derivative", "Integrator");
        List<String> logic = Arrays.asList("And", "Equals", "Greater", "GreaterEquals", "Less", "LessEquals", "Not",
                "Or", "Switch");

        extractList(basePath, "arithmetic", arithmetic);
        extractList(basePath, "basicLibrary", basicLibrary);
        extractList(basePath, "differential", differential);
        extractList(basePath, "logic", logic);
    }

    private static void extractList(String basePath, String packageName, List<String> files) {
        String path = String.join(".", basePath, packageName).replace(".","/");

        for (String file : files) {
            File res = getFile(String.join("/", path, String.join(".", file, "emam")));
            if (res == null || !res.exists())
                Log.error(String.format("Could not extract file \"%s\" from the basic library",
                        String.join(".", packageName, file)));
        }
    }

    private static File getFile(String file) {
        Path temp = Paths.get(String.join("/", BASIC_LIBRARY_ROOT, file));
        try {
            InputStream resourceAsStream = Thread.currentThread().getContextClassLoader().getResourceAsStream(file);
            if (resourceAsStream == null || temp == null) return new File("");
            temp.getParent().toFile().mkdirs();
            Files.copy(resourceAsStream, temp, StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException e) {
            return new File("");
        }

        return temp.toFile();
    }
}
