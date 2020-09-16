package de.monticore.lang.monticar.utilities.utils;

import com.google.common.base.Preconditions;
import org.apache.maven.project.MavenProject;
import org.apache.maven.shared.invoker.*;

import java.util.Collections;
import java.util.Properties;

public class JarDeployer {

  private static final String FILE = "file";
  private static final String URL = "url";
  private static final String GROUP_ID = "groupId";
  private static final String ARTIFACT_ID = "artifactId";
  private static final String VERSION = "version";

  public static void deployArtifact(String jarFile, MavenProject project) throws MavenInvocationException {
    nullCheck(project);

    Properties properties = new Properties();
    properties.setProperty(FILE, jarFile);
    properties.setProperty(URL, "file:/home/abdallah/.m2/test-repository");
    properties.setProperty(GROUP_ID, project.getGroupId());
    properties.setProperty(ARTIFACT_ID, project.getArtifactId());
    properties.setProperty(VERSION, project.getVersion());

    InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("deploy:deploy-file"));
    request.setProperties(properties);

    Invoker invoker = new DefaultInvoker();
    invoker.execute(request);
  }

  private static void nullCheck(MavenProject project) {
    Preconditions.checkNotNull(project.getGroupId(), "GroupId of the project must be specified");
    Preconditions.checkNotNull(project.getArtifactId(), "ArtifactId of the project must be specified");
    Preconditions.checkNotNull(project.getVersion(), "Version of the project must be specified");
  }


}
