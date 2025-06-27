/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.file;

import com.google.common.collect.Lists;
import de.se_rwth.commons.logging.Log;

import java.io.*;
import java.net.URISyntaxException;
import java.nio.file.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static com.google.common.base.Preconditions.checkNotNull;

/**
 *
 */
public class WrapperFileCreator {
    private final static List<String> ARMANPY_FILES = Lists.newArrayList(
            "armanpy.hpp",
            "armanpy.i",
            "armanpy_1d.i",
            "armanpy_2d.i",
            "armanpy_3d.i",
            "numpy.i"
    );

    private static final String ARMANPY_PATH = "de/monticore/lang/monticar/generator/pythonwrapper/file/armanpy";

    private String outputDirectory;

    private String getOutputDirectory() {
        return outputDirectory;
    }

    public void setOutputDirectory(String outputDirectory) {
        this.outputDirectory = outputDirectory;
    }

    public List<File> createWrapperFiles(final Map<String, String> fileContentMap) throws IOException {
        checkNotNull(outputDirectory);
        checkNotNull(fileContentMap);
        List<File> allCreatedFiles = new ArrayList<>();
        List<File> generatedFiles = createFromFileContentMap(fileContentMap);
        List<File> armanpyFiles = createArmanpyFiles();
        allCreatedFiles.addAll(generatedFiles);
        allCreatedFiles.addAll(armanpyFiles);
        return allCreatedFiles;
    }

    private List<File> createFromFileContentMap(Map<String, String> fileContentMap) throws IOException {
        List<File> files = new ArrayList<>();
        for (Map.Entry<String, String> fileContent : fileContentMap.entrySet()) {
            File f = createFileFromGeneratedContent(fileContent.getKey(), fileContent.getValue());
            files.add(f);
        }
        return files;
    }

    private File createFileFromGeneratedContent(String fileName, String generatedContent) throws IOException {
        File f = new File(Paths.get(getOutputDirectory(), fileName).toString());
        Log.info(f.getName(), "FileCreation:");
        if (!f.exists()) {
            f.getParentFile().mkdirs();
            if(!f.createNewFile()) {
                Log.error("File could not be created");
            }
        }
        BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(f));
        bufferedWriter.write(generatedContent, 0, generatedContent.length());
        bufferedWriter.close();
        return f;
    }

    private List<File> createArmanpyFiles() throws IOException {
        List<File> armanpyFiles = new ArrayList<>();

        final Path targetPath = Paths.get(getOutputDirectory(), "armanpy");

        if (!targetPath.toFile().exists()) {
            targetPath.toFile().mkdirs();
        }

        final String targetDirectory = targetPath.toString();

        CopyOption[] copyOptions = new CopyOption[] {
                StandardCopyOption.REPLACE_EXISTING
        };
        for (String armanpyFile : ARMANPY_FILES) {
            final String sourceJavaPath = ARMANPY_PATH + "/" + armanpyFile;
            final Path target = Paths.get(targetDirectory, armanpyFile);
            InputStream sourceInputStream = this.getClass().getClassLoader().getResourceAsStream(sourceJavaPath);
            Files.copy(sourceInputStream, target, copyOptions);
            Log.info(armanpyFile, "FileCreation:");
            armanpyFiles.add(target.toFile());
        }
        return armanpyFiles;
    }
}
