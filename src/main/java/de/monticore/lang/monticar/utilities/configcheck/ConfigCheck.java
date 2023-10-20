package de.monticore.lang.monticar.utilities.configcheck;

import java.io.*;
import java.util.*;
import com.google.gson.Gson;
import de.monticore.lang.monticar.utilities.artifactinmporter.ArtifactImporter;
import de.monticore.lang.monticar.utilities.models.StorageInformation;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import org.apache.maven.model.*;
import org.apache.maven.model.io.xpp3.MavenXpp3Reader;
import org.apache.maven.profiles.Profile;
import org.apache.maven.shared.invoker.*;
import org.codehaus.plexus.util.xml.pull.XmlPullParserException;

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
        String filter = "modelToTrain";
        ArtifactManager.getArtifacts(filter);

        // TODO: Find similar runs

        return configurationAlreadyRun;
    }

    public void deployConfigCheckArtifact(File settingsFile) {
        createConfFile();
        DeploymentRepository repository = getGitlabRepository();
        ArtifactManager.deployArtifact(getStorageInformation(), repository, settingsFile);
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
        try {
            FileWriter writer = new FileWriter(pathTmp + "/" + encoded + ".json");
            new Gson().toJson(configurationMap, writer);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
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

    private DeploymentRepository getGitlabRepository() {
        DeploymentRepository repository = null;
        File pluginPomFile = new File("pom.xml");
        MavenXpp3Reader reader = new MavenXpp3Reader();
        try {
            Model pluginPomModel = reader.read(new FileReader(pluginPomFile));
            List profiles = pluginPomModel.getProfiles();
            System.out.println("PROFILES:");
            for (Object p : profiles) {
                Profile profile = ((Profile) p);
                System.out.println(profile.getId());
                if (profile.getId().equals("gitlab-maven")) {
                    Repository repo = (Repository) profile.getRepositories().get(0);
                    repository.setId(repo.getId());
                    repository.setUrl(repo.getUrl());
                }
            }
        } catch (IOException | XmlPullParserException e) {
            e.printStackTrace();
        }
        return repository;
    }
}
