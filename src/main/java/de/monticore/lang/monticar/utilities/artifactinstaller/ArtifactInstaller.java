package de.monticore.lang.monticar.utilities.artifactinstaller;

import de.monticore.lang.monticar.utilities.models.Repository;
import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;

import java.util.Collections;
import java.util.Properties;

public class ArtifactInstaller {

  public static void installArtifact(Dependency dependency) throws MavenInvocationException {
    Properties properties = new Properties();
    properties.setProperty("outputDirectory", "${project.build.directory}/emadl-environment");
    properties.setProperty("artifact",String.format("%s:%s:%s", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion()) );

    InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:unpack"));
    request.setProperties(properties);

    Invoker invoker = new DefaultInvoker();
    invoker.execute(request);
  }

}
