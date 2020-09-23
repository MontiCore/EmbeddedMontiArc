package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.commons.lang3.StringUtils;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
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

    EMADLParser emadlParser = new EMADLParser();

    Optional<ASTEMACompilationUnit> emadlUnit = emadlParser.parse(modelPath.getAbsolutePath());
    if (!emadlUnit.isPresent()) {
      throw new MojoExecutionException("No parser for " + modelPath.getAbsolutePath() + " found");
    }
    ASTEMACompilationUnit emaAST = emadlUnit.get();

    String cnnTrainLocation = String.format("%s%s%s%s", modelPath.getParent(), File.separator, emaAST.getComponent().getName(), ".cnnt");
    File cnnTrainPath = new File(cnnTrainLocation);
    if (!cnnTrainPath.exists()) {
      throw new MojoExecutionException("CNNTrain File " + cnnTrainLocation + " not found");
    }

    String packagePath = getPackagePath(emaAST.getPackageList());

    Manifest manifest = createManifest(modelGroupId, modelArtifactId, modelToStore.getVersion());
    String jarFileName = createJarFileName(tempDirectory, "model");
    List<FileLocation> fileLocations = getFileLocations(packagePath, modelPath, cnnTrainPath);

    return JarCreator.createArtifact(jarFileName, manifest, fileLocations);
  }

  private static String getPackagePath(List<String> packageList) {
    return packageList.isEmpty() ? "" : String.join(File.separator, packageList) + File.separator;
  }

  private static List<FileLocation> getFileLocations(String packagePath, File modelPath, File cnnTrainPath) throws MojoExecutionException, IOException {
    FileLocation modelLocation = new FileLocation();
    modelLocation.setSourceLocation(modelPath.getAbsolutePath());
    modelLocation.setJarLocation(String.format("model%s%s%s", File.separator, packagePath, modelPath.getName()));

    FileLocation cnnLocation = new FileLocation();
    cnnLocation.setSourceLocation(cnnTrainPath.getAbsolutePath());
    cnnLocation.setJarLocation(String.format("model%s%s%s", File.separator, packagePath, cnnTrainPath.getName()));

    return new LinkedList<FileLocation>() { {add(modelLocation); add(cnnLocation);} };
  }

}

