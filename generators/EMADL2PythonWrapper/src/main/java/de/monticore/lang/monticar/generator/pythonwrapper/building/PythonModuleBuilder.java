/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.pythonwrapper.building;

import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.*;

import static com.google.common.base.Preconditions.*;

public class PythonModuleBuilder {
    private static final String OS = System.getProperty("os.name").toLowerCase();

    public void buildPythonModule(final Path sourceDirectory, final String componentName,
                                  final Path outputDirectory)
            throws PythonModuleBuildingException {
        checkState(checkPythonModuleBuildAvailable(), "Cannot build module: OS not supported");
        checkNotNull(outputDirectory);
        checkNotNull(sourceDirectory);
        checkNotNull(componentName);
        checkState(Paths.get(sourceDirectory.toString(), "CMakeLists.txt").toFile().exists(),
                "No cmake file in source directory");

        Path buildDirectory;
        try {
            buildDirectory = makeBuildDirectory(sourceDirectory);
            buildPythonWrapperModule(buildDirectory, sourceDirectory);
            moveModuleToOutputDirectory(buildDirectory, componentName, outputDirectory);
        } catch (IOException | InterruptedException e) {
            throw new PythonModuleBuildingException("Cannot create build directory: " + e.getMessage());
        }
    }

    public boolean checkPythonModuleBuildAvailable() {
        return (OS.contains("mac") || OS.contains("nix") || OS.contains("nux") || OS.contains("aix"));
    }

    private void moveModuleToOutputDirectory(
            final Path buildDirectory,
            final String componentName,
            final Path outputDirectory) throws IOException {
        checkState(buildDirectory.toFile().exists());

        final Path pythonFile = Paths.get(buildDirectory.toString(), componentName + "_executor.py");
        final Path libraryFile = Paths.get(buildDirectory.toString(), "_" + componentName + "_executor.so");

        checkState(pythonFile.toFile().exists(), "Building failed: No python module found");
        checkState(libraryFile.toFile().exists(), "Building failed: No library module found");

        if (!outputDirectory.toFile().exists()) {
            outputDirectory.toFile().mkdirs();
        }

        final Path outputPythonFile = Paths.get(outputDirectory.toString(), componentName + "_executor.py");
        final Path outputLibraryFile = Paths.get(outputDirectory.toString(), "_" + componentName + "_executor.so");

        Log.info("Copy python module " + componentName + "_executor.py to location " + outputPythonFile.toString(),
                "Reward Function Builder");
        Files.copy(pythonFile, outputPythonFile, StandardCopyOption.REPLACE_EXISTING);
        Log.info("Copy python module " + componentName + "_executor.py to location " + outputLibraryFile.toString(),
                "Reward Function Builder");
        Files.copy(libraryFile, outputLibraryFile, StandardCopyOption.REPLACE_EXISTING);
    }

    private void buildPythonWrapperModule(final Path buildDirectory, final Path sourceDirectory)
            throws IOException, InterruptedException, PythonModuleBuildingException {
        final String fullPathToSourceDirectory = sourceDirectory.toString();
        final String fullPathToBuildDirectory = buildDirectory.toString();
        final String[] cmakeCommand = {"cmake", "-B" + fullPathToBuildDirectory,
                "-H" + fullPathToSourceDirectory};
        final String[] makeCommand = {"make", "-C", fullPathToBuildDirectory};

        ProcessBuilder cmakeProcessBuilder = new ProcessBuilder(cmakeCommand);
        ProcessBuilder makeProcessBuilder = new ProcessBuilder(makeCommand);
        Log.info("Building reward function....", "Reward Function Builder");
        Process p = cmakeProcessBuilder.start();
        String cmakeOutput = readProcessErrorOutput(p);
        int errCode = p.waitFor();

        if (errCode == 0) {
            p = makeProcessBuilder.start();
            String makeOutput = readProcessErrorOutput(p);
            errCode = p.waitFor();

            if (errCode != 0) {
                throw new PythonModuleBuildingException(String.format("Make command failed: %s", makeOutput));
            }
        } else {
            throw new PythonModuleBuildingException(String.format("CMake command failed: %s", cmakeOutput));
        }

        Log.info("Building finished", "Reward Function Builder");
    }

    private String readProcessErrorOutput(final Process p) throws IOException {
        BufferedReader reader = new BufferedReader(new InputStreamReader(p.getErrorStream()));
        StringBuilder outputBuilder = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            outputBuilder.append(line);
            outputBuilder.append(System.getProperty("line.separator"));
        }
        return outputBuilder.toString();
    }

    private Path makeBuildDirectory(final Path sourceDirectory) throws IOException, PythonModuleBuildingException {
        final Path buildPath = Paths.get(sourceDirectory.toString(), "build");
        final File buildDirectoryFile = buildPath.toFile();

        if (buildDirectoryFile.exists() && buildDirectoryFile.isDirectory()) {
            FileUtils.deleteDirectory(buildDirectoryFile);
        }

        boolean creationSuccess = buildDirectoryFile.mkdirs();
        if (!creationSuccess) {
            throw new PythonModuleBuildingException("Cannot create directory");
        }
        return buildPath;
    }
}
