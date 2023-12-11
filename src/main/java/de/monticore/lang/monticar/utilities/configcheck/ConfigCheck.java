package de.monticore.lang.monticar.utilities.configcheck;

import de.monticore.lang.monticar.utilities.models.StorageInformation;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;
import java.io.File;
import java.util.UUID;

public class ConfigCheck {
    private static String uuid = UUID.randomUUID().toString();
    private String groupId = "artifacts-1";
    private String artifactId;
    private String version;
    private String experimentName;
    private String gitlabPackagesUrl;
    private String gitlabAccessToken;
    private String gitlabArtifactsPath;
    private String mlflowTrackingUrl;
    private String pathTmp;

    public String getGitlabPackagesUrl() {
        return gitlabPackagesUrl;
    }

    public String getGitlabHost() {
        return gitlabPackagesUrl != null ? gitlabPackagesUrl.split("/api/")[0] : null;
    }

    public String getGitlabAccessToken() {
        return gitlabAccessToken;
    }

    public String getGitlabProjectId() {
        if (gitlabPackagesUrl != null) {
            String[] parts = gitlabPackagesUrl.split("/");
            for (String part : parts) {
                if (part.matches("\\d+")) {
                    return part;
                }
            }
        }
        return null;
    }

    public String getMlflowTrackingUrl() {
        return mlflowTrackingUrl;
    }

    public String getPathTmp() {
        return pathTmp != null ? pathTmp : "target/tmp/config-check";
    }

    public String getExperimentName() {
        if (experimentName != null && !experimentName.isEmpty()) {
            return experimentName;
        }
        return uuid;
    }

    public String getGroupId() { return groupId; }

    public String getArtifactId() { return artifactId; }

    public String getPackageName() { return String.format("%s--experiment-%s", artifactId, getExperimentName()); }

    public String getVersion() { return version; }

    public String getGitlabArtifactsPath() { return gitlabArtifactsPath != null ? gitlabArtifactsPath : String.format("%s/gitlabArtifacts", getPathTmp()); }

    public void setArtifactId(String artifactId_) {
        artifactId = artifactId_;
    }

    public void setVersion(String version_) {
        version = version_;
    }

    public void setGitlabPackagesUrl(String gitlabPackagesUrl_) {
        gitlabPackagesUrl = gitlabPackagesUrl_;
    }

    public void setGitlabAccessToken(String gitlabAccessToken_) {
        gitlabAccessToken = gitlabAccessToken_;
    }

    public void setGitlabArtifactsPath(String gitlabArtifactsPath_) {
        gitlabArtifactsPath = gitlabArtifactsPath_;
    }

    public Dependency getDependency() {
        Dependency dependency = new Dependency();
        dependency.setGroupId(groupId);
        dependency.setArtifactId(artifactId);
        dependency.setVersion(version);
        return dependency;
    }

    public StorageInformation getStorageInformation() {
        StorageInformation storageInformation = new StorageInformation();
        storageInformation.setGroupId(groupId);
        storageInformation.setArtifactId(getPackageName());
        storageInformation.setVersion(version);
        storageInformation.setPath(new File(String.format("%s/%s", getPathTmp(), getExperimentName())));
        return storageInformation;
    }

    public DeploymentRepository getGitlabRepository() {
        DeploymentRepository deploymentRepository = new DeploymentRepository();
        deploymentRepository.setId("gitlab-maven");
        deploymentRepository.setUrl(gitlabPackagesUrl);
        return deploymentRepository;
    }
}
