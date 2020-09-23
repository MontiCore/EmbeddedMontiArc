package de.monticore.lang.monticar.utilities;

import de.monticore.lang.monticar.utilities.models.Constants;
import de.monticore.lang.monticar.utilities.models.Repository;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.maven.plugin.AbstractMojo;
import org.apache.maven.plugins.annotations.Component;
import org.apache.maven.plugins.annotations.Parameter;
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

public abstract class BaseMojo extends AbstractMojo {

  @Component
  private RepositorySystem repositorySystem;

  @Parameter( defaultValue = "${repositorySystemSession}", readonly = true )
  private RepositorySystemSession repositorySystemSession;

  @Parameter
  private Repository repository;

  private RemoteRepository remoteRepository;

  protected static final String TEMP_FOLDER = "target/tmp";

  public void mkTmpDir() {
    this.mkdir(TEMP_FOLDER);
  }

  private void mkdir(String path) {
    try {
      File tmpOut = Paths.get(path).toFile();
      if(!tmpOut.exists()){
        tmpOut.mkdirs();
      }
    }catch (Exception ex){
      ex.printStackTrace();
    }
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

    Artifact artifact = new DefaultArtifact( String.format("%s:%s:[1,)" , storageInformation.getGroupId(), storageInformation.getArtifactId()));

    VersionRangeRequest rangeRequest = new VersionRangeRequest();
    rangeRequest.setArtifact( artifact );
    rangeRequest.setRepositories(Collections.singletonList(getRemoteRepository()));

    int newestVersion;
    try {
      VersionRangeResult rangeResult = repositorySystem.resolveVersionRange( repositorySystemSession, rangeRequest );
      newestVersion = Integer.parseInt(rangeResult.getHighestVersion().toString());
      return ++newestVersion;
    }
    catch (VersionRangeResolutionException e) {
      e.printStackTrace();
    }

    return Constants.INITIAL_VERSION;
  }

}
