package de.monticore.lang.monticar.utilities.artifactdeployer;

import de.monticore.lang.monticar.utilities.models.Repository;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarDeployer;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.util.Properties;

public class ArtifactDeployer {

  private static final String FILE = "file";
  private static final String REPOSITORY_ID = "repositoryId";
  private static final String URL = "url";
  private static final String GROUP_ID = "groupId";
  private static final String ARTIFACT_ID = "artifactId";
  private static final String VERSION = "version";

  public static void deployArtifact(String jarFile, StorageInformation datasetToStore, Repository repository) throws MavenInvocationException {
    Properties properties = new Properties();
    properties.setProperty(FILE, jarFile);
    properties.setProperty(REPOSITORY_ID, repository.getId());
    properties.setProperty(URL, repository.getUrl().toString());
    properties.setProperty(GROUP_ID, datasetToStore.getGroupId());
    properties.setProperty(ARTIFACT_ID, datasetToStore.getArtifactId());
    properties.setProperty(VERSION, String.valueOf(datasetToStore.getVersion()));

    JarDeployer.deployArtifact(properties);
  }

}
