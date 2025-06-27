package de.monticore.lang.monticar.utilities.mojos;

import de.monticore.lang.monticar.utilities.models.Constants;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.maven.execution.MavenSession;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugin.BuildPluginManager;
import org.apache.maven.plugin.MojoExecutionException;
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
import java.util.Collections;
import java.util.List;

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

  /**
   * Target directory where all generated components are stored and created JARs are temporarily
   * cached before being deployed. <br>
   */
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

  public DeploymentRepository getRepository() {
    return mavenProject.getDistributionManagement().getRepository();
  }

  public List<Dependency> getDependencies() {
    return mavenProject.getDependencies();
  }

  private RemoteRepository getRemoteRepository() {
    if (remoteRepository != null) {
      return remoteRepository;
    }

    remoteRepository = new RemoteRepository.Builder(getRepository().getId(), "default", getRepository().getUrl()).build();
    return remoteRepository;
  }

  public String getNewestVersion(StorageInformation storageInformation) throws MojoExecutionException {
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
      return String.valueOf(++newestVersion);
    }
    catch (VersionRangeResolutionException | NullPointerException e) {
      e.printStackTrace();
      throw new MojoExecutionException("Version was not defined and could not be determined automatically.");
    }

  }
}
