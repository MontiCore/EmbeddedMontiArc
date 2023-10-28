package de.monticore.lang.monticar.utilities.configcheck;

import de.monticore.lang.monticar.utilities.artifactinmporter.ArtifactImporter;
import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Properties;

public class ConfigCheckArtifactImporter extends ArtifactImporter{

    public static void importArtifact(Dependency dependency, File targetPath) throws MavenInvocationException {
        Properties properties = new Properties();
        properties.setProperty("outputDirectory", targetPath.getAbsolutePath() + "/runConfigurations");
        properties.setProperty("artifact", String.format("%s:%s:%s:jar", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion()));

        InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:unpack"));
        request.setProperties(properties);
        request.setProfiles(getGitlabProfile());
        Invoker invoker = new DefaultInvoker();
        invoker.execute(request);
    }

    private static List<String> getGitlabProfile() {
        List<String> profiles = new ArrayList<>();
        profiles.add("gitlab-maven");
        return profiles;
    }
}
