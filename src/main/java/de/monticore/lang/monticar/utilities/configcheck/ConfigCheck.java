package de.monticore.lang.monticar.utilities.configcheck;

import java.io.*;
import java.util.*;
import com.google.gson.Gson;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import org.gitlab4j.api.models.Package;

public class ConfigCheck {
    protected TrainingConfiguration trainingConfiguration;
    protected Map<String, String> configurationMap;
    protected String pathTmp;

    public ConfigCheck(TrainingConfiguration trainingConfiguration, String pathTmp) {
        this.trainingConfiguration = trainingConfiguration;
        this.configurationMap = ConfigurationParser.parseConfiguration(trainingConfiguration);
        this.pathTmp = pathTmp;
    }

    public boolean configurationAlreadyRun() {
        boolean configurationAlreadyRun = false;
        Gson gson = new Gson();
        GitlabPackagesManager gitlabManager = new GitlabPackagesManager();
        String configurationString = gson.toJson(configurationMap);


        // query database
        String filter = "modelToTrain"; // or self.project_name
        List<Package> packages = gitlabManager.getPackages();

        // TODO: Find similar runs
        System.out.println("PACKAGES:");
        for (Package pkg : packages) {
            System.out.println(pkg.getId());
            System.out.println(pkg.getName());
        }

        return configurationAlreadyRun;
    }

    public void deployArtifact(String version, File settingsFile) {
        createConfFile();
        ConfigCheckArtifactDeployer.deployArtifact(getStorageInformation(version), settingsFile);
    }

    private void createConfFile() {
        // TODO: this should be removed once the training works
        createFoldersIfNotExists(pathTmp);
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
        Gson gson = new Gson();
        String encoded = Encoder.encode(gson.toJson(configurationMap.values()));
        version = (version == null || version.isEmpty()) ? "1.0.0" : version;

        StorageInformation storageInformation = new StorageInformation();
        storageInformation.setGroupId(String.format("config-check_%s", configurationMap.get("modelToTrain")));
        storageInformation.setArtifactId(encoded);
        storageInformation.setVersion(version);
        storageInformation.setPath(new File(pathTmp));

        return storageInformation;
    }
}
