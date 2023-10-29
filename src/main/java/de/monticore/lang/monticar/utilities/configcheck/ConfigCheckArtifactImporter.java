package de.monticore.lang.monticar.utilities.configcheck;

import de.monticore.lang.monticar.utilities.artifactinmporter.ArtifactImporter;
import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Properties;

public class ConfigCheckArtifactImporter extends ArtifactImporter{

    public static void importArtifact(Dependency dependency, File targetPath) throws MavenInvocationException {
//        enableGitlabProfile();
        Properties properties = new Properties();
        properties.setProperty("artifact", String.format("%s:%s:%s", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion()));
        properties.setProperty("remoteRepositories", "https://git.rwth-aachen.de/api/v4/projects/49355/packages/maven");

        InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:get"));
        request.setProperties(properties);
//        request.setProfiles(getGitlabProfile());
        Invoker invoker = new DefaultInvoker();
        invoker.execute(request);
//        disableGitlabProfile();
    }

    private static void enableGitlabProfile() {
        try {
            FileWriter writer = new FileWriter("target/tmp/useGitlabProfile.temp");
            writer.write(1);
            writer.close();
            System.out.println("created useGitlabProfile.temp");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void disableGitlabProfile() {
        File tempFile = new File("target/tmp/useGitlabProfile.temp");
        if (tempFile.exists()) {
            boolean res = tempFile.delete();
            System.out.println("deleted useGitlabProfile: " + res);
        }
    }

    private static List<String> getGitlabProfile() {
        List<String> profiles = new ArrayList<>();
        profiles.add("gitlab-maven");
        return profiles;
    }
}
