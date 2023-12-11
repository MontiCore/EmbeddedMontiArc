package de.monticore.lang.monticar.utilities.configcheck;

import de.se_rwth.commons.logging.Log;
import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;
import org.gitlab4j.api.GitLabApi;
import org.gitlab4j.api.GitLabApiException;
import org.gitlab4j.api.PackagesApi;
import org.gitlab4j.api.models.Package;
import java.io.*;
import java.util.*;

public class ConfigCheckArtifactImporter {

    public static void importArtifact(Dependency dependency, File targetPath, File userSettingsFile) throws MavenInvocationException {
        String artifact = String.format("%s:%s:%s:jar", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion());
        Properties properties = new Properties();
        properties.setProperty("outputDirectory", targetPath.getAbsolutePath());
        properties.setProperty("artifact", artifact);

        Log.info(String.format("[ConfigCheck] Importing artifact '%s' into %s", artifact, targetPath.getPath()), "[ConfigCheck]");
        InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:unpack"));
        request.setProperties(properties);
        request.setUserSettingsFile(userSettingsFile);
        Invoker invoker = new DefaultInvoker();
        invoker.execute(request);
    }

    public static void importSimilarArtifacts(ConfigCheck configCheck, File targetPath, File userSettingsFile) {
        String projectId = configCheck.getGitlabProjectId();
        String regexFilter = String.format("%s/%s--experiment-.*", configCheck.getGroupId(), configCheck.getArtifactId());
        PackagesApi packagesApi = null;
        try (GitLabApi gitLabApi = new GitLabApi(configCheck.getGitlabHost(), configCheck.getGitlabAccessToken())) {
            packagesApi = gitLabApi.getPackagesApi();
        } catch (Exception e) {
            Log.info(String.format("[ConfigCheck] Could not connect to GitLab: %s", e.getMessage()), "[ConfigCheck]");
        }

        if (packagesApi == null || projectId == null) {
            Log.info("[ConfigCheck] Importing similar artifacts failed: packagesApi or projectId not found", "[ConfigCheck]");
            return;
        }

        List<Package> packageList = new ArrayList<>();
        try {
            packageList = packagesApi.getPackages(projectId);
        } catch (GitLabApiException  e) {
            Log.error(String.format("[ConfigCheck] GitLabApiException while trying to import similar artifacts. %s", e.getMessage()));
        }
        Log.info(String.format("Iterating gitlab packages for projectId='%s': [regexFilter='%s']", projectId, regexFilter), "[ConfigCheck]");
        for (Package gitlabPackage : packageList) {
            // Find all the packages matching groupId/modelToTrain.*    (artifactId = [modelToTrain/UUID])
            if (gitlabPackage.getName().matches(regexFilter) && gitlabPackage.getVersion().equals(configCheck.getVersion())) {
                Dependency dependency = new Dependency();
                dependency.setGroupId(gitlabPackage.getName().split("/")[0]);
                dependency.setArtifactId(gitlabPackage.getName().split("/")[1]);
                dependency.setVersion(gitlabPackage.getVersion());
                Log.info(String.format("Importing dependency %s:%s:%s", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion()), "[ConfigCheck]");
                String experimentName = gitlabPackage.getName().split("--experiment-")[1];
                File targetPath_ = new File(String.format("%s/%s", targetPath, experimentName));
                try {
                    importArtifact(dependency, targetPath_, userSettingsFile);
                } catch (MavenInvocationException e) {
                    Log.error(String.format("[ConfigCheck] MavenInvocationException while trying to import similar artifacts. %s", e.getMessage()));
                }
            }
        }
    }
}
