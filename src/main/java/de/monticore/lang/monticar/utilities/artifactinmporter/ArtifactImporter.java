package de.monticore.lang.monticar.utilities.artifactinmporter;

import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;

import java.io.File;
import java.util.Collections;
import java.util.Properties;

public class ArtifactImporter {

  public static void importArtifact(Dependency dependency, File targetPath) throws MavenInvocationException {
    Properties properties = new Properties();
    properties.setProperty("outputDirectory", targetPath.getAbsolutePath());
    properties.setProperty("artifact",String.format("%s:%s:%s:jar:%s",
            dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion(), dependency.getClassifier()));

    InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:unpack"));
    request.setProperties(properties);

    Invoker invoker = new DefaultInvoker();
    invoker.execute(request);
  }

}
