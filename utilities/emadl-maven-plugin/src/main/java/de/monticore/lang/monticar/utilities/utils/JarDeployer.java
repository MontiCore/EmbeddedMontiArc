package de.monticore.lang.monticar.utilities.utils;

import de.monticore.lang.monticar.utilities.artifactdeployer.POMGenerator;
import org.apache.maven.shared.invoker.*;

import java.io.File;
import java.util.Collections;
import java.util.Properties;

public class JarDeployer {

  public static void deployArtifact(Properties properties, File settingsFile) throws MavenInvocationException {
    InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("deploy:deploy-file"));
    request.setProperties(properties);
    request.setUserSettingsFile(settingsFile);
    Invoker invoker = new DefaultInvoker();
    invoker.execute(request);
  }

  public static void installArtifact(Properties properties) throws MavenInvocationException {
    InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("install:install-file"));
    request.setProperties(properties);

    Invoker invoker = new DefaultInvoker();
    invoker.execute(request);
  }

}
