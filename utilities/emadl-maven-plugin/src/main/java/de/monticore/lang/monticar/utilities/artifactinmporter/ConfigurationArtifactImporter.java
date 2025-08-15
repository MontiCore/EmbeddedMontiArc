package de.monticore.lang.monticar.utilities.artifactinmporter;

import de.monticore.lang.monticar.utilities.utils.ConfigurationTracking;
import de.monticore.mlpipelines.util.configuration_tracking.ConfigurationTrackingConf;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.model.Dependency;
import org.apache.maven.shared.invoker.*;
import org.gitlab4j.api.GitLabApi;
import org.gitlab4j.api.GitLabApiException;
import org.gitlab4j.api.PackagesApi;
import org.gitlab4j.api.models.Package;
import org.gitlab4j.api.models.PackageFilter;
import java.io.*;
import java.util.*;
import java.util.stream.Collectors;

public class ConfigurationArtifactImporter {

    public static void importArtifact(Dependency dependency, File targetPath, File userSettingsFile) throws MavenInvocationException {
        String artifact = String.format("%s:%s:%s:jar", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion());
        Properties properties = new Properties();
        properties.setProperty("outputDirectory", targetPath.getAbsolutePath());
        properties.setProperty("artifact", artifact);

        Log.info(String.format("Importing artifact '%s' into %s", artifact, targetPath.getPath()), ConfigurationArtifactImporter.class.getName());
        InvocationRequest request = new DefaultInvocationRequest().setGoals(Collections.singletonList("dependency:unpack"));
        request.setProperties(properties);
        request.setUserSettingsFile(userSettingsFile);
        Invoker invoker = new DefaultInvoker();
        invoker.execute(request);
    }

    public static void importGitlabArtifacts(File userSettingsFile) {
        String projectId = ConfigurationTracking.getGitlabProjectId();
        String filter = String.format("%s/%s--experiment-", ConfigurationTracking.getGroupId(), ConfigurationTracking.getArtifactId());
        PackagesApi packagesApi = null;
        if (ConfigurationTracking.getGitlabHost() != null) {
            try (GitLabApi gitLabApi = new GitLabApi(ConfigurationTracking.getGitlabHost(), ConfigurationTracking.getGitlabAccessToken())) {
                packagesApi = gitLabApi.getPackagesApi();
            } catch (Exception e) {
                Log.info(String.format("Could not connect to GitLab: %s", e.getMessage()), ConfigurationArtifactImporter.class.getName());
            }
        }

        if (packagesApi == null || projectId == null) {
            Log.info("Importing similar artifacts failed: packagesApi or projectId not found", ConfigurationArtifactImporter.class.getName());
            return;
        }

        List<Package> packageList = new ArrayList<>();
        try {
            packageList = packagesApi.getPackagesStream(projectId, new PackageFilter().withPackageName(filter))
                    .filter(artifact -> artifact.getVersion().equals(ConfigurationTracking.getVersion()))
                    .collect(Collectors.toList());
        } catch (GitLabApiException  e) {
            Log.error(String.format("GitLabApiException while trying to import artifacts. %s", e.getMessage()));
        }

        for (Package gitlabPackage : packageList) {
            Dependency dependency = new Dependency();
            dependency.setGroupId(gitlabPackage.getName().split("/")[0]);
            dependency.setArtifactId(gitlabPackage.getName().split("/")[1]);
            dependency.setVersion(gitlabPackage.getVersion());
            String experimentName = gitlabPackage.getName().split("--experiment-")[1];
            File targetPath_ = new File(String.format("%s/%s", ConfigurationTrackingConf.getGitlabArtifactsPath(), experimentName));
            try {
                importArtifact(dependency, targetPath_, userSettingsFile);
            } catch (MavenInvocationException e) {
                Log.error(String.format("MavenInvocationException while trying to import artifacts. %s", e.getMessage()));
            }
        }
    }
}
