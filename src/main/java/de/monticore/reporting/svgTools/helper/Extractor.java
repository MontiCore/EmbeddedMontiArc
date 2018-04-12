package de.monticore.reporting.svgTools.helper;

import org.apache.commons.io.FileUtils;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

public class Extractor {

    /**
     * Copy a file from source to destination.
     *
     * @param source
     *        the source
     * @param destination
     *        the destination
     * @return True if succeeded , False if not
     */
    private static boolean copy(InputStream source , String destination) {
        boolean succeess = true;

        System.out.println("Copying ->" + source + "\n\tto ->" + destination);

        try {
            Files.copy(source, Paths.get(destination), StandardCopyOption.REPLACE_EXISTING);
        } catch (IOException ex) {
            succeess = false;
        }

        return succeess;

    }

    public static void extractSVGGenerator() {
        Extractor extractor = new Extractor();
        try {
            extractor.extract();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void extract() throws IOException {
        List<String> iconsToCopy = new ArrayList<>();
        iconsToCopy.add("svggenerator.jar");

        for (String fileName : iconsToCopy) {
            URL inputUrl = ClassLoader.getSystemResource(fileName);
            File targetParent = new File("svgGenerator");
            if(!targetParent.exists()) targetParent.mkdirs();
            File target = Paths.get("svgGenerator/" + fileName).toFile();
            System.out.println("copying svgGenerator " + inputUrl + " to " + target.getAbsolutePath());
            FileUtils.copyURLToFile(inputUrl, target);
            targetParent.deleteOnExit();
            target.deleteOnExit();
        }
    }
}
