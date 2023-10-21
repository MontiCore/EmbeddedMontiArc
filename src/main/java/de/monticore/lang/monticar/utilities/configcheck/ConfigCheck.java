package de.monticore.lang.monticar.utilities.configcheck;

import java.io.*;
import java.util.*;
import com.google.gson.Gson;
import de.monticore.lang.monticar.utilities.artifactinmporter.ArtifactImporter;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import org.apache.maven.model.*;
import org.apache.maven.shared.invoker.*;

public class ConfigCheck {
    protected TrainingConfiguration trainingConfiguration;
    protected Map<String, String> configurationMap;
    protected String pathTmp;

    public ConfigCheck(TrainingConfiguration trainingConfiguration, String pathTmp) {
        this.trainingConfiguration = trainingConfiguration;
        this.configurationMap = new ConfigurationParser().parseConfiguration(trainingConfiguration);
        this.pathTmp = pathTmp;
    }

//    private String generateEncodedArchitectureString() {
//        if (this.parsedPipelineConfiguration == null) {
//            return null;
//        }
//
//        // TODO: Better encoding function ?
//        byte[] encodedBytes = Base64.getEncoder().encode(this.parsedPipelineConfiguration.getBytes());
//        return new String(encodedBytes);
//    }
//
    public boolean configurationAlreadyRun() {
        boolean configurationAlreadyRun = false;
        Gson gson = new Gson();
        String configurationString = gson.toJson(configurationMap);


        // query database
        String filter = "modelToTrain"; // or self.project_name

        // TODO: Find similar runs

        return configurationAlreadyRun;
    }

    public void deployArtifact(File settingsFile) {
        createConfFile();
        ConfigCheckArtifactDeployer.deployArtifact(getStorageInformation(), settingsFile);
    }

    private void importConfigCheckArtifact(String importPath) {
        Dependency dependency = new Dependency();
        dependency.setGroupId("de.monticore.lang.monticar.utilities");
        dependency.setArtifactId("config-check");
        dependency.setVersion("1");
        File targetPath = new File(importPath);

        try {
            ArtifactImporter.importArtifact(dependency, targetPath);
            System.out.println("Artifact imported in " + importPath);
        } catch (MavenInvocationException e) {
            e.printStackTrace();
        }
    }

    private void createConfFile() {
//        String encoded = Encoder.encode(gson.toJson(configurationMap));
        String encoded = "ENCODED";
        // TODO: this should be removed once the training works
        createFoldersIfNotExists(pathTmp);
        try {
            FileWriter writer = new FileWriter(pathTmp + "/" + encoded + ".json");
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

    private StorageInformation getStorageInformation() {
        Gson gson = new Gson();
//        String encoded = Encoder.encode(gson.toJson(configurationMap));
        String encoded = "ENCODED";

        StorageInformation storageInformation = new StorageInformation();
//        storageInformation.setGroupId("de.monticore.lang.monticar.utilities");
        storageInformation.setGroupId("config-check");
//        storageInformation.setArtifactId("config-check");
        storageInformation.setArtifactId(encoded);
        storageInformation.setVersion("1.0.0");
        storageInformation.setPath(new File(pathTmp));

        return storageInformation;
    }
}
