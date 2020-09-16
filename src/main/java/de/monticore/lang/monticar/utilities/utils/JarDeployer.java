package de.monticore.lang.monticar.utilities.utils;

import org.apache.maven.shared.invoker.*;

import java.util.Collections;
import java.util.Properties;

public class JarDeployer {

  public static void deployArtifact(Properties properties) throws MavenInvocationException {
    InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("deploy:deploy-file"));
    request.setProperties(properties);

    Invoker invoker = new DefaultInvoker();
    invoker.execute(request);
  }

}
