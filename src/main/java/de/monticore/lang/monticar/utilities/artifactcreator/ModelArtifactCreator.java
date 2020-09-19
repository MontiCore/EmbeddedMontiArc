package de.monticore.lang.monticar.utilities.artifactcreator;

import com.google.common.base.Preconditions;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.FileLocation;
import de.monticore.lang.monticar.utilities.utils.JarCreator;
import org.apache.commons.lang3.StringUtils;
import org.apache.maven.plugin.MojoExecutionException;

import java.io.File;
import java.io.IOException;
import java.util.Collections;
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

    EMADLParser parser = new EMADLParser();
    Optional<ASTEMACompilationUnit> node = parser.parse(modelPath.getAbsolutePath());
    if (!node.isPresent()) {
      throw new MojoExecutionException("No parser for " + modelPath + " found");
    }
    ASTEMACompilationUnit ast = node.get();

    Manifest manifest = createManifest(modelGroupId, modelArtifactId);
    String jarFileName = createJarFileName(tempDirectory, "model");
    String packagePath = createPackagePath(modelPath.getName(), ast.getPackageList());

    FileLocation fileLocation = new FileLocation();
    fileLocation.setJarLocation(String.format("model%s%s", File.separator, packagePath));
    fileLocation.setSourceLocation(modelPath.getAbsolutePath());

    return JarCreator.createArtifact(jarFileName, manifest, Collections.singletonList(fileLocation));
  }

  private static String createPackagePath(String name, List<String> packageList) {
    String packageName = packageList.isEmpty() ? "" : String.join(File.separator, packageList) + File.separator;
    return packageName + name;
  }

}

