package de.monticore.lang.monticar.utilities.artifactdeployer;

import de.monticore.lang.monticar.utilities.models.Repository;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
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
  private static final String CLASSIFIER = "classifier";
  private static final String PACKAGING = "packaging";

  public static void deployArtifact(String jarFile, StorageInformation storageInformation, Repository repository, JarClassifierEnum classifier)
      throws MavenInvocationException {
    Properties properties = getDeployProperties(jarFile, storageInformation, repository, classifier);

    JarDeployer.deployArtifact(properties);
  }

  public static void installArtifact(String jarFile, StorageInformation storageInformation, Repository repository, JarClassifierEnum classifier)
      throws MavenInvocationException {
    Properties properties = getInstallProperties(jarFile, storageInformation, classifier);

    JarDeployer.installArtifact(properties);
  }

  private static Properties getDeployProperties(String jarFile, StorageInformation storageInformation, Repository repository, JarClassifierEnum classifier) {
    Properties properties = new Properties();
    properties.setProperty(REPOSITORY_ID, repository.getId());
    properties.setProperty(URL, repository.getUrl().toString());
    return getProperties(jarFile, storageInformation, classifier, properties);
  }

  private static Properties getInstallProperties(String jarFile, StorageInformation storageInformation, JarClassifierEnum classifier) {
    Properties properties = new Properties();
    properties.setProperty(PACKAGING, "jar");
    return getProperties(jarFile, storageInformation, classifier, properties);
  }

  private static Properties getProperties(String jarFile, StorageInformation storageInformation, JarClassifierEnum classifier, Properties properties) {
    properties.setProperty(FILE, jarFile);
    properties.setProperty(GROUP_ID, storageInformation.getGroupId());
    properties.setProperty(ARTIFACT_ID, storageInformation.getArtifactId());
    properties.setProperty(VERSION, String.valueOf(storageInformation.getVersion()));
    properties.setProperty(CLASSIFIER, classifier.value);
    return properties;
  }

}
