package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.commons.io.FilenameUtils;
import org.apache.commons.lang3.StringUtils;
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
    String modelGroupId = modelToStore.getGroupId();
    String modelArtifactId = modelToStore.getArtifactId();
    File modelPath = modelToStore.getPath();
    Preconditions.checkArgument(!StringUtils.isEmpty(modelGroupId), "Group ID of EMADL model artifact must be specified.");
    Preconditions.checkArgument(!StringUtils.isEmpty(modelArtifactId), "Artifact ID of EMADL model artifact must be specified.");
    Preconditions.checkNotNull(modelPath, "Path of EMADL model must be specified.");

    Manifest manifest = createManifest(modelGroupId, modelArtifactId, modelToStore.getVersion());
    String jarFileName = createJarFileName(tempDirectory, "model");
    List<FileLocation> fileLocations = getFileLocations(modelToStore);

    return JarCreator.createArtifact(jarFileName, manifest, fileLocations);
  }

  private static String getPackagePath(List<String> packageList) {
    return packageList.isEmpty() ? "" : String.join(File.separator, packageList) + File.separator;
  }

  private static List<FileLocation> getFileLocations(StorageInformation storageInformation) throws IOException {
    String extension;
    List<FileLocation> modelLocations = new LinkedList<>();

    EMADLParser emadlParser = new EMADLParser();

    for (File file: Objects.requireNonNull(storageInformation.getPath().listFiles())) {
      if (!file.isFile() && file.isDirectory()) {
        StorageInformation subdirectoryInformation = storageInformation.createCopy();
        subdirectoryInformation.setPath(file);
        modelLocations.addAll(getFileLocations(subdirectoryInformation));
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
    File cnntFile = new File(String.format("%s%s", FilenameUtils.removeExtension(model.getAbsolutePath()), ".cnnt"));
    if (!cnntFile.exists()) {
      return null;
    }

    return createFileLocation(packagePath, cnntFile);
  }

  private static FileLocation createFileLocation(String packagePath, File model) {
    FileLocation modelLocation = new FileLocation();
    modelLocation.setSourceLocation(model.getAbsolutePath());
    modelLocation.setJarLocation(String.format("model%s%s%s", File.separator, packagePath, model.getName()));

    return modelLocation;
  }

}

