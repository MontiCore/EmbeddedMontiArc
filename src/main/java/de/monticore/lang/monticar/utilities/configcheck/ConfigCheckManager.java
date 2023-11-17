package de.monticore.lang.monticar.utilities.configcheck;

import java.io.*;
import java.util.*;
import com.google.gson.Gson;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.shared.invoker.MavenInvocationException;
import org.mlflow.tracking.MlflowClient;

public class ConfigCheckManager {
    private final String CONFIG_CHECK_FILE_PATH = "src/main/resources/config-check.json";
    private final ConfigCheck configCheck;
    private final MlflowClient mlflowClient;
    protected Map<String, String> configurationMap;
    protected File settingsFile;
    protected String experimentUuid;

    public ConfigCheckManager(TrainingConfiguration trainingConfiguration, File settingsFile) {
        this.experimentUuid = UUID.randomUUID().toString();
        this.configCheck = readConfigCheckFile();
        this.mlflowClient = new MlflowClient(this.configCheck.getMlflowTrackingUrl());
        this.configurationMap = ConfigurationParser.parseConfiguration(trainingConfiguration);
        this.settingsFile = settingsFile;
        mkdirs(this.configCheck.getPathTmp());
    }

    private static void mkdirs(String folderPath) {
        File folder = new File(folderPath);
        if (!folder.exists()) {
            folder.mkdirs();
        }
    }

    private void createConfFile() {
        try {
            FileWriter writer = new FileWriter(configCheck.getPathTmp() + "/runConfigurations/" + experimentUuid + "-config-check.json");
            new Gson().toJson(configurationMap, writer);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private StorageInformation getStorageInformation(String version) {
        StorageInformation storageInformation = new StorageInformation();
        storageInformation.setGroupId("config-check");
        storageInformation.setArtifactId(configurationMap.get("modelToTrain"));
        storageInformation.setVersion(version);
        storageInformation.setPath(new File(configCheck.getPathTmp() + "/runConfigurations"));
        return storageInformation;
    }

    private Dependency getDependency(String version) {
        Dependency dependency = new Dependency();
        dependency.setGroupId("config-check");
        dependency.setArtifactId(configurationMap.get("modelToTrain"));
        dependency.setVersion(version);
        return dependency;
    }

    private List<Map<String, String>> getRunConfigurationsGitlab() {
        List<Map<String, String>> runConfigurations = new ArrayList<>();
        File[] files = new File(configCheck.getPathTmp() + "/runConfigurations").listFiles();

        if (files != null) {
            for (File jsonConf : files) {
                if (!jsonConf.isFile()) {
                    continue;
                }
                try {
                    FileReader reader = new FileReader(jsonConf.getPath());
                    Gson gson = new Gson();
                    Map<String, String> configuration = gson.fromJson(reader, Map.class);
                    if (configuration != null) {
                        runConfigurations.add(configuration);
                    }
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }
            }
        }
        System.out.printf("Got %d run configurations from GitLab\n", runConfigurations.size());
        return runConfigurations;
    }

    private List<Map<String, String>> getRunConfigurationsMlflow() {
        List<Map<String, String>> runConfigurations = new ArrayList<>();
        // TODO: implement search
//        mlflowClient.searchRuns("");
        System.out.printf("Got %d run configurations from Mlflow\n", runConfigurations.size());
        return runConfigurations;
    }

    private DeploymentRepository getGitlabRepository() {
        DeploymentRepository deploymentRepository = new DeploymentRepository();
        deploymentRepository.setId("gitlab-maven");
        deploymentRepository.setUrl(configCheck.getGitlabPackagesUrl());
        return deploymentRepository;
    }

    public void importArtifact(String version) {
        Dependency dependency = getDependency(version);
        try {
            ConfigCheckArtifactImporter.importArtifact(dependency, new File(configCheck.getPathTmp()), settingsFile);
        } catch (MavenInvocationException e) {
            e.printStackTrace();
        }
    }

    public void deployArtifact(String version) {
        createConfFile();
        ConfigCheckArtifactDeployer.deployArtifact(getStorageInformation(version), getGitlabRepository(), settingsFile);
    }

    public boolean configurationAlreadyRun() {
        for (Map<String, String> conf : getRunConfigurationsGitlab()) {
            if (conf.equals(configurationMap)) {
                return true;
            }
        }

        for (Map<String, String> conf : getRunConfigurationsMlflow()) {
            if (conf.equals(configurationMap)) {
                return true;
            }
        }
        return false;
    }

    public boolean isEnabled() {
        return configCheck != null;
    }

    private ConfigCheck readConfigCheckFile() {
        try (Reader reader = new FileReader(CONFIG_CHECK_FILE_PATH)) {
            Gson gson = new Gson();
            return gson.fromJson(reader, ConfigCheck.class);
        } catch (Exception e) {
            System.out.printf("[ConfigCheck] Failed reading %s: %s\n", CONFIG_CHECK_FILE_PATH, e.getMessage());
        }
        return null;
    }
}
