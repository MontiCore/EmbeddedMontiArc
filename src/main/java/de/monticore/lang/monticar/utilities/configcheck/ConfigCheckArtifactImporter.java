package de.monticore.lang.monticar.utilities.configcheck;

import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;
import java.io.File;
import java.util.Collections;
import java.util.Properties;

public class ConfigCheckArtifactImporter {

    public static void importArtifact(Dependency dependency, File targetPath, File userSettingsFile) throws MavenInvocationException {
        Properties properties = new Properties();
        properties.setProperty("outputDirectory", targetPath.getAbsolutePath() + "/runConfigurations");
        properties.setProperty("artifact", String.format("%s:%s:%s:jar", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion()));

        InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:unpack"));
        request.setProperties(properties);
        request.setUserSettingsFile(userSettingsFile);
        Invoker invoker = new DefaultInvoker();
        invoker.execute(request);
    }
}
