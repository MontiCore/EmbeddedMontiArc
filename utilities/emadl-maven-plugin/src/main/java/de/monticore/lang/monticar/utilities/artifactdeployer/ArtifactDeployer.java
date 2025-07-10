package de.monticore.lang.monticar.utilities.artifactdeployer;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.utils.JarClassifierEnum;
import de.monticore.lang.monticar.utilities.utils.JarDeployer;
import freemarker.template.Configuration;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.shared.invoker.MavenInvocationException;

import java.io.File;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
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
  private static final String POMFILE = "pomFile";

  public static void deployArtifact(String jarFile, StorageInformation storageInformation, DeploymentRepository repository, JarClassifierEnum classifier, File settingsFile, List<Dependency> dependencies)
      throws MavenInvocationException {
    Properties properties = getDeployProperties(jarFile, storageInformation, repository, classifier, dependencies);
    JarDeployer.deployArtifact(properties, settingsFile);
  }

  public static void deployArtifact(String jarFile, StorageInformation storageInformation, DeploymentRepository repository, JarClassifierEnum classifier, File settingsFile)
          throws MavenInvocationException {
    deployArtifact(jarFile, storageInformation, repository, classifier, settingsFile,  new LinkedList<>());
  }

  public static void installArtifact(String jarFile, StorageInformation storageInformation, JarClassifierEnum classifier)
      throws MavenInvocationException {
    installArtifact(jarFile, storageInformation, classifier,  new LinkedList<>());

  }

  public static void installArtifact(String jarFile, StorageInformation storageInformation, JarClassifierEnum classifier, List<Dependency> dependencies)
          throws MavenInvocationException {
    Properties properties = getInstallProperties(jarFile, storageInformation, classifier, dependencies);
    JarDeployer.installArtifact(properties);
  }

  private static Properties getDeployProperties(String jarFile, StorageInformation storageInformation, DeploymentRepository repository, JarClassifierEnum classifier, List<Dependency> dependencies) {
    File pomFile;
    try {
      pomFile = POMGenerator.generatePOM(storageInformation, repository, dependencies);
    } catch (IOException e){
      e.printStackTrace();
      throw new RuntimeException("Could not generate POM.");
    }

    Properties properties = new Properties();
    properties.setProperty(REPOSITORY_ID, repository.getId());
    properties.setProperty(URL, repository.getUrl());
    properties.setProperty(POMFILE, pomFile.getPath());
    return getProperties(jarFile, storageInformation, classifier, properties);
  }

  private static Properties getInstallProperties(String jarFile, StorageInformation storageInformation, JarClassifierEnum classifier, List<Dependency> dependencies) {
    File pomFile;
    try {
      pomFile = POMGenerator.generatePOM(storageInformation, null, dependencies);
    } catch (IOException e){
      e.printStackTrace();
      throw new RuntimeException("Could not generate POM.");
    }

    Properties properties = new Properties();
    properties.setProperty(PACKAGING, "jar");
    properties.setProperty(POMFILE, pomFile.getPath());
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
