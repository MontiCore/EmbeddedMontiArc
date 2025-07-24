package de.monticore.lang.monticar.utilities.artifactcreator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.commons.io.FilenameUtils;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.jar.Manifest;

public class ModelArtifactCreator extends ArtifactCreator {

  public static File createArtifact(StorageInformation modelToStore, String tempDirectory) throws IOException, MojoExecutionException {
    checkStorageInformation(modelToStore, "EMADL project");
    String modelGroupId = modelToStore.getGroupId();
    String modelArtifactId = modelToStore.getArtifactId();
    File modelPath = modelToStore.getPath();

    Manifest manifest = createManifest(modelGroupId, modelArtifactId, modelToStore.getVersion());
    String jarFileName = createJarFileName(tempDirectory, "model");

    EMADLParser emadlParser = new EMADLParser();
    List<FileLocation> fileLocations = getFileLocations(modelPath, emadlParser);

    return JarCreator.createArtifact(jarFileName, manifest, fileLocations);
  }

  protected static String getPackagePath(List<String> packageList) {
    return packageList.isEmpty() ? "" : String.join(File.separator, packageList) + File.separator;
  }

  protected static List<FileLocation> getFileLocations(File modelPath, EMADLParser emadlParser) throws IOException {
    String extension;
    List<FileLocation> modelLocations = new LinkedList<>();

    for (File file : Objects.requireNonNull(modelPath.listFiles())) {
      if (!file.isFile() && file.isDirectory()) {
        modelLocations.addAll(getFileLocations(file, emadlParser));
      }

      extension = FilenameUtils.getExtension(file.getName());

      String packagePath;
      if ("emadl".equals(extension)) {
        Optional<ASTEMACompilationUnit> emadlUnit = emadlParser.parse(file.getAbsolutePath());
        ASTEMACompilationUnit emaAST = emadlUnit.get();
        packagePath = getPackagePath(emaAST.getPackageList());
        modelLocations.add(createFileLocation(packagePath, file));

        FileLocation cnntLocation = createCNNTFileLocation(packagePath, file);
        if (cnntLocation != null) {
          modelLocations.add(cnntLocation);
        }
      }

    }

    return modelLocations;
  }

  private static FileLocation createCNNTFileLocation(String packagePath, File model) {
    File cnntFile = new File(String.format("%s%s", FilenameUtils.removeExtension(model.getAbsolutePath()), ".conf"));
    if (!cnntFile.exists()) {
      return null;
    }

    return createFileLocation(packagePath, cnntFile);
  }

  private static FileLocation createFileLocation(String packagePath, File model) {
    FileLocation modelLocation = new FileLocation();
    modelLocation.setSourceLocation(model.getAbsolutePath());
    modelLocation.setJarLocation(packagePath + model.getName());

    return modelLocation;
  }

}