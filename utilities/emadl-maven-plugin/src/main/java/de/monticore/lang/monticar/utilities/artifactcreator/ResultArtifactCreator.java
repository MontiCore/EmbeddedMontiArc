package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.apache.commons.lang3.StringUtils;
import org.apache.maven.plugin.MojoExecutionException;
import org.apache.maven.plugin.MojoFailureException;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.*;
import java.util.jar.Attributes;
import java.util.jar.Manifest;

public class ResultArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation storageInformation, TrainingConfiguration trainingConfiguration, String tmpOut, TaggingResolver taggingResolver)
      throws MojoFailureException, MojoExecutionException, IOException {
    checkStorageInformation(storageInformation, "training result");
    EMAComponentSymbol component = (EMAComponentSymbol) taggingResolver.resolve(trainingConfiguration.getModelToTrain(), EMAComponentSymbol.KIND).orElse(null);
    if (component == null) {
      throw new MojoFailureException(String.format("Component with name %s does not exist", trainingConfiguration.getModelToTrain()));
    }

    String instanceName = component.getName().substring(0, 1).toLowerCase() + component.getName().substring(1);
    Optional<EMAComponentInstanceSymbol> instanceSymbolOptional = component.getEnclosingScope().resolve(instanceName, EMAComponentInstanceSymbol.KIND);
    String fileName = instanceSymbolOptional.get().getFullName().replaceAll("\\.", "_");

    List<FileLocation> fileLocations = getResultFiles(tmpOut, fileName, component.getFullName());
    String jarFileName = createJarFileName(tmpOut, "result");
    Manifest manifest = createManifest(storageInformation.getGroupId(), storageInformation.getArtifactId(), storageInformation.getVersion(), getMetric(component.getFullName()));

    return JarCreator.createArtifact(jarFileName, manifest, fileLocations);
  }

  private static List<FileLocation> getResultFiles(String path, String fileName, String componentName) throws MojoExecutionException {
    List<FileLocation> fileLocations = new LinkedList<>();

    File directory = new File(path + "/cpp/" + componentName + "/");
    List<File> files = Arrays.asList(directory.listFiles(getFileNameFilter(fileName)));
    if (files.stream().noneMatch(file -> file.getName().equals(String.format("%s.cpp", fileName))) &&
        files.stream().noneMatch(file -> file.getName().equals(String.format("%s.h", fileName)))) {
      throw new MojoExecutionException(String.format("Files %s.cpp and %s.h not available.", fileName, fileName));
    }

    for (File file : files) {
      FileLocation fileLocation = new FileLocation();
      fileLocation.setSourceLocation(file.getAbsolutePath());
      fileLocation.setJarLocation(String.format("component%s%s", File.separator, file.getName()));
      fileLocations.add(fileLocation);
    }

    String modelDirectory = String.format("%s/model/%s/", System.getProperty("user.dir"), componentName);
    File architectureFile = new File(modelDirectory, "model_0_newest-symbol.json");
    if (!architectureFile.exists()) {
      throw new MojoExecutionException(String.format("Architecture file %s not found.", architectureFile.getAbsolutePath()));
    }
    else {
      FileLocation architectureLocation = new FileLocation();
      architectureLocation.setSourceLocation(architectureFile.getAbsolutePath());
      architectureLocation.setJarLocation(String.format("model%s%s%s%s", File.separator, componentName, File.separator, architectureFile.getName()));
      fileLocations.add(architectureLocation);
    }

    File paramsFile = new File(modelDirectory, "model_0_newest-0000.params");
    if (!paramsFile.exists()) {
      throw new MojoExecutionException(String.format("Params file %s not found.", paramsFile.getAbsolutePath()));
    }
    else {
      FileLocation paramsLocation = new FileLocation();
      paramsLocation.setSourceLocation(paramsFile.getAbsolutePath());
      paramsLocation.setJarLocation(String.format("model%s%s%s%s", File.separator, componentName, File.separator, paramsFile.getName()));
      fileLocations.add(paramsLocation);
    }

    return fileLocations;
  }

  private static Attributes getMetric(String componentName) throws FileNotFoundException {
    String modelDirectory = String.format("%s/model/%s/", System.getProperty("user.dir"), componentName);
    File metricFile = new File(modelDirectory + File.separator + "metric.txt");
    Attributes attributes = new Attributes();

    Scanner scanner = new Scanner(metricFile);
    if (!scanner.hasNextLine()) {
      return attributes;
    }

    String[] metric = scanner.nextLine().split(" ");
    attributes.put(new Attributes.Name(metric[0].substring(0, 1).toUpperCase() + metric[0].substring(1)), metric[1]);
    return attributes;
  }

  private static FilenameFilter getFileNameFilter(String fileName) {
    return (dir, name) ->
        name.endsWith(fileName + ".cpp") ||
        name.endsWith(fileName + ".h") ||
        StringUtils.equals(name, "CNNBufferFile.h") ||
        StringUtils.equals(name, "CNNTranslator.h") ||
        StringUtils.equals(name, "CNNModelLoader.h") ||
        StringUtils.equals(name, "HelperA.h");
  }

}
