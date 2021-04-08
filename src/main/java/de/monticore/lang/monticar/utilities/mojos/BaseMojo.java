package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.generator.order.nfp.TagBreakpointsTagSchema.TagBreakpointsTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagDelayTagSchema.TagDelayTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagInitTagSchema.TagInitTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagMinMaxTagSchema.TagMinMaxTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagTableTagSchema.TagTableTagSchema;
import de.monticore.lang.monticar.generator.order.nfp.TagThresholdTagSchema.TagThresholdTagSchema;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.monticar.utilities.models.Constants;
import de.monticore.lang.monticar.utilities.models.Repository;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import org.apache.maven.execution.MavenSession;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.BuildPluginManager;
import org.apache.maven.plugins.annotations.Component;
import org.apache.maven.plugins.annotations.Parameter;
import org.apache.maven.project.MavenProject;
import org.eclipse.aether.RepositorySystem;
import org.eclipse.aether.RepositorySystemSession;
import org.eclipse.aether.artifact.Artifact;
import org.eclipse.aether.artifact.DefaultArtifact;
import org.eclipse.aether.repository.RemoteRepository;
import org.eclipse.aether.resolution.VersionRangeRequest;
import org.eclipse.aether.resolution.VersionRangeResolutionException;
import org.eclipse.aether.resolution.VersionRangeResult;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;

public abstract class BaseMojo extends AbstractMojo {

  @Parameter(defaultValue = "${repositorySystem}", readonly = true)
  private RepositorySystem repositorySystem;

  @Parameter( defaultValue = "${project}", readonly = true )
  private MavenProject mavenProject;

  @Parameter( defaultValue = "${session}", readonly = true )
  private MavenSession mavenSession;

  @Component
  private BuildPluginManager pluginManager;

  @Parameter(defaultValue = "${repositorySystemSession}", readonly = true)
  private RepositorySystemSession repositorySystemSession;

  @Parameter
  private Repository repository;

  @Parameter(property = "pathTmpOut", defaultValue = "target/tmp")
  private String pathTmpOut;

  private RemoteRepository remoteRepository;

  public void mkTmpDir() {
    this.mkdir(getPathTmpOut());
  }

  private void mkdir(String path) {
    try {
      File tmpOut = Paths.get(path).toFile();
      if (!tmpOut.exists()) {
        tmpOut.mkdirs();
      }
    }
    catch (Exception ex) {
      ex.printStackTrace();
    }
  }

  public MavenProject getMavenProject() {
    return mavenProject;
  }

  public MavenSession getMavenSession() {
    return mavenSession;
  }

  public BuildPluginManager getPluginManager() {
    return pluginManager;
  }

  public String getPathTmpOut() {
    return pathTmpOut;
  }

  public Repository getRepository() {
    return repository;
  }

  public RepositorySystem getRepositorySystem() {
    return repositorySystem;
  }

  public RepositorySystemSession getRepositorySystemSession() {
    return repositorySystemSession;
  }

  public RemoteRepository getRemoteRepository() {
    if (remoteRepository != null) {
      return remoteRepository;
    }

    remoteRepository = new RemoteRepository.Builder(repository.getId(), "default", repository.getUrl().getPath()).build();
    return remoteRepository;
  }

  public int getNewestVersion(StorageInformation storageInformation) {
    if (storageInformation.getVersion() != null) {
      return storageInformation.getVersion();
    }

    Artifact artifact = new DefaultArtifact(String.format("%s:%s:[1,)", storageInformation.getGroupId(), storageInformation.getArtifactId()));

    VersionRangeRequest rangeRequest = new VersionRangeRequest();
    rangeRequest.setArtifact(artifact);
    rangeRequest.setRepositories(Collections.singletonList(getRemoteRepository()));

    int newestVersion;
    try {
      VersionRangeResult rangeResult = repositorySystem.resolveVersionRange(repositorySystemSession, rangeRequest);
      newestVersion = rangeResult.getHighestVersion() != null ? Integer.parseInt(rangeResult.getHighestVersion().toString()) : Constants.INITIAL_VERSION - 1;
      return ++newestVersion;
    }
    catch (VersionRangeResolutionException e) {
      e.printStackTrace();
    }

    return Constants.INITIAL_VERSION;
  }
}
