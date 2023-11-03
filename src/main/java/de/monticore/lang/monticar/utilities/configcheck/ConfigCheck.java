package de.monticore.lang.monticar.utilities.configcheck;

import java.io.*;
import java.util.*;
import com.google.gson.Gson;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import org.apache.maven.model.Dependency;
import org.apache.maven.model.DeploymentRepository;
import org.apache.maven.shared.invoker.MavenInvocationException;

public class ConfigCheck {
    private static final String GITLAB_API_URL = "https://git.rwth-aachen.de/";
    private static final String PROJECT_ID = "49355";
    protected TrainingConfiguration trainingConfiguration;
    protected Map<String, String> configurationMap;
    protected String pathTmp;
    protected File settingsFile;

    public ConfigCheck(TrainingConfiguration trainingConfiguration, String pathTmp, File settingsFile) {
        this.trainingConfiguration = trainingConfiguration;
        this.configurationMap = ConfigurationParser.parseConfiguration(trainingConfiguration);
        this.pathTmp = pathTmp;
        this.settingsFile = settingsFile;
        // TODO: this should be removed once the training works
        createFoldersIfNotExists(pathTmp);
    }

    public void importArtifact(String version, File targetPath) {
        Dependency dependency = getDependency(version);
        try {
            ConfigCheckArtifactImporter.importArtifact(dependency, targetPath, settingsFile);
        } catch (MavenInvocationException e) {
            e.printStackTrace();
        }
    }

    public void deployArtifact(String version) {
        createConfFile();
        ConfigCheckArtifactDeployer.deployArtifact(getStorageInformation(version), settingsFile);
    }

    public boolean configurationAlreadyRun() {
        for (Map<String, String> conf : getRunConfigurations()) {
            if (conf.equals(configurationMap)) {
                return true;
            }
        }
        return false;
    }

    private void createConfFile() {
        try {
            FileWriter writer = new FileWriter(pathTmp + "/config-check.json");
            new Gson().toJson(configurationMap, writer);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void createFoldersIfNotExists(String folderPath) {
        File folder = new File(folderPath);
        if (!folder.exists()) {
            folder.mkdirs();
        }
    }

    private StorageInformation getStorageInformation(String version) {
        StorageInformation storageInformation = new StorageInformation();
        storageInformation.setGroupId("config-check");
        storageInformation.setArtifactId(configurationMap.get("modelToTrain"));
        storageInformation.setVersion(version);
        storageInformation.setPath(new File(pathTmp));
        return storageInformation;
    }

    private Dependency getDependency(String version) {
        Dependency dependency = new Dependency();
        dependency.setGroupId("config-check");
        dependency.setArtifactId(configurationMap.get("modelToTrain"));
        dependency.setVersion(version);
        return dependency;
    }

    private List<Map<String, String>> getRunConfigurations() {
        List<Map<String, String>> runConfigurations = new ArrayList<>();
        File[] files = new File(pathTmp + "/runConfigurations").listFiles();

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
        System.out.printf("Got %d run configurations\n", runConfigurations.size());
        return runConfigurations;
    }

    public static DeploymentRepository getGitlabRepository() {
        // TODO: Read the url from settingsFile?
        DeploymentRepository deploymentRepository = new DeploymentRepository();
        deploymentRepository.setId("gitlab-maven");
        deploymentRepository.setUrl(GITLAB_API_URL + "api/v4/projects/" + PROJECT_ID + "/packages/maven");
        return deploymentRepository;
    }
}
