package de.monticore.lang.monticar.utilities.utils;

import java.io.*;
import java.util.*;

import de.monticore.lang.monticar.utilities.artifactinmporter.ConfigurationArtifactImporter;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.monticore.mlpipelines.util.configuration_tracking.ConfigurationTrackingConf;
import de.monticore.parsing.ConfigurationLanguageParser;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.execution.MavenSession;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.project.MavenProject;
import org.apache.maven.settings.Server;
import org.apache.maven.settings.Settings;
import org.codehaus.plexus.util.xml.Xpp3Dom;

public class ConfigurationTracking {
    private static final String DATASET_FORMAT = "%s:%s:%s";
    private static String groupId = "CAR";
    private static String artifactId;
    private static String version;
    private static String gitlabAccessToken;

    public static boolean initialize(String artifactTrackingConfiguration, MavenProject project, MavenSession session, TrainingConfiguration trainingConfiguration) {
        try {
            ConfigurationTrackingConf.fromASTConfLangCompilationUnit(new ConfigurationLanguageParser().parseModelIfExists(artifactTrackingConfiguration));
            String groupId_ = ConfigurationTrackingConf.getGroupId();
            if (groupId_ != null && !groupId_.isEmpty()) {
                groupId = groupId_;
            }
        } catch (IOException e) {
            Log.error(String.format("Invalid or missing file for ConfigCheck: `%s`", artifactTrackingConfiguration));
        }
        artifactId = trainingConfiguration.getModelToTrain();
        version = project.getVersion();
        gitlabAccessToken = extractGitLabAccessToken(session.getSettings());
        createRequiredDirs();
        trackDatasetDependency(project.getDependencies());
        ConfigurationArtifactImporter.importGitlabArtifacts(session.getRequest().getUserSettingsFile());
        return checkUniqueName();
    }

    public static String getGitlabHost() {
        return ConfigurationTrackingConf.getGitlabTrackingUrl() != null ? ConfigurationTrackingConf.getGitlabTrackingUrl().split("/api/")[0] : null;
    }

    public static String getGitlabAccessToken() {
        return gitlabAccessToken;
    }

    public static String getGitlabProjectId() {
        if (ConfigurationTrackingConf.getGitlabTrackingUrl() != null) {
            String[] parts = ConfigurationTrackingConf.getGitlabTrackingUrl().split("/");
            for (String part : parts) {
                if (part.matches("\\d+")) {
                    return part;
                }
            }
        }
        return null;
    }

    public static String getGroupId() { return groupId; }

    public static String getArtifactId() { return artifactId; }

    public static String getVersion() { return version; }

    public static String getPackageName() { return String.format("%s--experiment-%s", artifactId, ConfigurationTrackingConf.getExperimentName()); }

    public static StorageInformation getStorageInformation() {
        StorageInformation storageInformation = new StorageInformation();
        storageInformation.setGroupId(groupId);
        storageInformation.setArtifactId(getPackageName());
        storageInformation.setVersion(version);
        storageInformation.setPath(new File(String.format("%s/%s", ConfigurationTrackingConf.getPathTmp(), ConfigurationTrackingConf.getExperimentName())));
        return storageInformation;
    }

    public static DeploymentRepository getGitlabRepository() {
        DeploymentRepository deploymentRepository = new DeploymentRepository();
        deploymentRepository.setId("gitlab-maven");
        deploymentRepository.setUrl(ConfigurationTrackingConf.getGitlabTrackingUrl());
        return deploymentRepository;
    }

    public static void trackDatasetDependency(List<Dependency> dependencies) {
        for (Dependency dependency : dependencies) {
            if (dependency.getGroupId().equals("de.embeddedmontiarcdl.datasets") || dependency.getGroupId().equals("de.monticore.lang.monticar.datasets")) {
                String dataset = String.format(DATASET_FORMAT, dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion());
                try {
                    FileWriter writer = new FileWriter(String.format("%s/%s", ConfigurationTrackingConf.getPathTmp(), ConfigurationTrackingConf.getDatasetFileName()));
                    writer.write(dataset);
                    writer.close();
                } catch (IOException e) {
                    Log.error(String.format("Could not save the dataset info into %s/datasetInfo.txt. Error: %s", ConfigurationTrackingConf.getPathTmp(), e.getMessage()));
                }
            }
        }
    }

    private static void createRequiredDirs() {
        File gitlabArtifactsPath = new File(ConfigurationTrackingConf.getGitlabArtifactsPath());
        File currentExperimentPath = new File(String.format("%s/%s", ConfigurationTrackingConf.getPathTmp(), ConfigurationTrackingConf.getExperimentName()));
        File artifactsDiffPath = new File(ConfigurationTrackingConf.getArtifactsDiffPath());
        if (!gitlabArtifactsPath.exists()) {
            Log.info(String.format("Creating %s", gitlabArtifactsPath.getPath()), ConfigurationTracking.class.getName());
            gitlabArtifactsPath.mkdirs();
        }
        if (!currentExperimentPath.exists()) {
            Log.info(String.format("Creating %s", currentExperimentPath.getPath()), ConfigurationTracking.class.getName());
            currentExperimentPath.mkdirs();
        }
        if (!artifactsDiffPath.exists()) {
            Log.info(String.format("Creating %s", artifactsDiffPath.getPath()), ConfigurationTracking.class.getName());
            artifactsDiffPath.mkdirs();
        }
    }

    private static String extractGitLabAccessToken(Settings settings) {
        if (settings != null && settings.getServers().size() > 0) {
            for (Server server : settings.getServers()) {
                if (server.getId().equals("gitlab-maven")) {
                    Xpp3Dom httpHeaders = ((Xpp3Dom) server.getConfiguration()).getChild("httpHeaders");
                    if (httpHeaders != null) {
                        Xpp3Dom property = httpHeaders.getChild("property");
                        if (property != null) {
                            String name = property.getChild("name").getValue();
                            String value = property.getChild("value").getValue();
                            // TODO: Add Deploy-Token and...?
                            if (name.equals("Private-Token")) {
                                return value;
                            }
                        }
                    }
                }
            }
        }
        return "ACCESS_TOKEN_NOT_FOUND";
    }

    public static boolean checkUniqueName() {
        File targetPath_ = new File(ConfigurationTrackingConf.getGitlabArtifactsPath());
        for (File experimentDir : Objects.requireNonNull(targetPath_.listFiles())) {
            if (experimentDir.getName().equals(ConfigurationTrackingConf.getExperimentName())) {
                return false;
            }
        }
        return true;
    }
}
