package de.monticore.lang.monticar.utilities.configcheck;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import de.monticore.lang.monticar.utilities.models.TrainingConfiguration;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.model.Dependency;
import org.apache.maven.settings.Server;
import org.apache.maven.settings.Settings;
import org.codehaus.plexus.util.xml.Xpp3Dom;
import org.mlflow.api.proto.Service;
import org.mlflow.tracking.ExperimentsPage;
import org.mlflow.tracking.MlflowClient;
import difflib.*;

public class ConfigCheckManager {
    private final String configurationFilePath = "src/main/resources/config-check.json";
    private ConfigCheck configCheck;
    private Map<String, String> configurationMap;

    public ConfigCheckManager(TrainingConfiguration trainingConfiguration, String version, Settings settings) {
        configCheck = readConfigCheckFile();
        configurationMap = ConfigurationParser.parseConfiguration(trainingConfiguration);
        if (configCheck != null) {
            configCheck.setArtifactId(configurationMap.get("modelToTrain"));
            configCheck.setVersion(version);
            configCheck.setGitlabAccessToken(extractGitLabAccessTokenFromSettings(settings));
            createRequiredDirs();
        }
    }

    public boolean isEnabled() {
        return configCheck != null;
    }

    private ConfigCheck readConfigCheckFile() {
        try (Reader reader = new FileReader(configurationFilePath)) {
            Gson gson = new GsonBuilder().setPrettyPrinting().disableHtmlEscaping().create();
            return gson.fromJson(reader, ConfigCheck.class);
        } catch (Exception e) {
            Log.info("ConfigCheck disabled: " + e.getMessage(), "CONFIG_CHECK");
            return null;
        }
    }

    public void importSimilarRuns(File settingsFile) {
        File targetPath = new File(configCheck.getGitlabArtifactsPath());
        ConfigCheckArtifactImporter.importSimilarArtifacts(configCheck, targetPath, settingsFile);
    }

    public void deployCurrentRuns(File settingsFile) {
        ConfigCheckArtifactDeployer.deployArtifact(configCheck.getStorageInformation(), configCheck.getGitlabRepository(), settingsFile);
    }

    public boolean configurationAlreadyRun() {
        System.out.printf("configurationMap: %s", new Gson().toJson(configurationMap));
        for (Map<String, String> conf : getGitlabArtifacts()) {
            double similarity = computeSimilarityForConfiguration(conf);
            if (similarity == 1) {
                Log.info("Past run with the same configuration found in the GitLab package registry" + conf, "CONFIG-CHECK");
                return true;
            }
            if (similarity == 0.5) {
                Log.info("Past run with similar configuration found in the GitLab package registry" + conf, "CONFIG-CHECK");
                return true;
            }
        }
        Log.info("No run with the same configuration was found in the GitLab package registry", "CONFIG-CHECK");

        for (Map<String, String> conf : getMlflowArtifacts()) {
            double similarity = computeSimilarityForConfiguration(conf);
            if (similarity == 1) {
                Log.info("Past run with the same configuration found in Mlflow" + conf, "CONFIG-CHECK");
                return true;
            }
            if (similarity == 0.5) {
                Log.info("Past run with similar configuration found in Mlflow" + conf, "CONFIG-CHECK");
                return true;
            }
        }
        Log.info("No run with the same configuration was found in Mlflow", "CONFIG-CHECK");
        return false;
    }

    private List<Map<String, String>> getGitlabArtifacts() {
        List<Map<String, String>> gitlabArtifacts = new ArrayList<>();
        File[] experimentDirs = new File(configCheck.getGitlabArtifactsPath()).listFiles();
        for (File experimentDir : Objects.requireNonNull(experimentDirs)) {
            for (File jsonConf : Objects.requireNonNull(experimentDir.listFiles())) {
                if (!jsonConf.isFile()) {
                    continue;
                }
                try {
                    FileReader reader = new FileReader(jsonConf.getPath());
                    Gson gson = new GsonBuilder().setPrettyPrinting().disableHtmlEscaping().create();
                    Map<String, String> configuration = gson.fromJson(reader, Map.class);
                    if (configuration != null) {
                        gitlabArtifacts.add(configuration);
                    }
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }
            }
        }
        Log.info(String.format("Got %d artifacts from GitLab", gitlabArtifacts.size()), "[ConfigCheck]");
        return gitlabArtifacts;
    }

    private List<Map<String, String>> getMlflowArtifacts() {
        MlflowClient mlflowClient = new MlflowClient(configCheck.getMlflowTrackingUrl());
        List<Map<String, String>> mlflowArtifacts = new ArrayList<>();

        ExperimentsPage experimentsPage = mlflowClient.searchExperiments();
        List<String> experimentIds = new ArrayList<>();
        for (Service.Experiment experiment : experimentsPage.getItems()) {
            experimentIds.add(experiment.getExperimentId());
        }

        // TODO: check this
        String searchFilter = String.format("tags.Model ILIKE '%s'", configurationMap.get("modelToTrain"));
//        String searchFilter = String.format("tags.Model ILIKE '%s' and tags.Version='%s'", configurationMap.get("modelToTrain"), configCheck.getVersion());
        List<Service.RunInfo> mlflowRuns = mlflowClient.searchRuns(experimentIds, searchFilter);

        for (Service.RunInfo runInfo : mlflowRuns) {
            // Skip the current run
            if (runInfo.getStatus().equals(Service.RunStatus.RUNNING)) {
                continue;
            }

            Service.Run run = mlflowClient.getRun(runInfo.getRunId());
            List<Service.Param> runParams = run.getData().getParamsList();
            Map<String, String> mlflowRunConfiguration = new TreeMap<>();
            for (Service.Param param : runParams) {
                mlflowRunConfiguration.put(param.getKey(), param.getValue());
            }

            File mlflowArtifact = mlflowClient.downloadArtifacts(runInfo.getRunId());
            try {
                String content = new String(Files.readAllBytes(Paths.get(String.format("%s/network.txt", mlflowArtifact.getPath()))));
                mlflowRunConfiguration.put("network", content);
            } catch (IOException e) {
                Log.error(String.format("[ConfigCheck] Could not load the content of the 'network.txt' artifact for Mlflow run %s: %s", runInfo.getRunId(), e.getMessage()));
                continue;
            }
            mlflowArtifacts.add(mlflowRunConfiguration);
        }

        Log.info(String.format("[ConfigCheck] Got %d artifacts from Mlflow", mlflowArtifacts.size()), "[ConfigCheck]");
        return mlflowArtifacts;
    }

    private static boolean similarNetworks(String network1, String network2) {
        // TODO: Check semantic differencing
        if (network1.equals(network2)) {
            return true;
        }
        return false;
    }

    private double computeSimilarityForConfiguration(Map<String, String> conf) {
        TreeMap<String, String> tempConfMap = new TreeMap<>(configurationMap);
        TreeMap<String, String> tempConf = new TreeMap<>(conf);
        TreeMap<String, String> difference = new TreeMap<>();
//
//        if (tempConfMap.size() < tempConf.size()) {
//            tempConfMap = new TreeMap<>(conf);
//            tempConf = new TreeMap<>(configurationMap);
//        }

        // TODO: network shouldn't be removed, but here I still don't have it (This is before the run has actually started)
        String[] keysToPop = new String[]{"train_accuracy", "modelToTrain", "backend", "run_at", "network", "name", "evaluation_step"};
        for (String keyToPop : keysToPop) {
            tempConfMap.remove(keyToPop);
            tempConf.remove(keyToPop);
        }
//
//        if (tempConfMap.equals(tempConf)) {
//            return 1;
//        }

        // TODO: Find what is similar and how relevant it is. Compare the network
        for (Map.Entry<String, String> entry : tempConfMap.entrySet()) {
            String key = entry.getKey();
            String value = entry.getValue();
            if (!tempConf.containsKey(key) || !tempConf.get(key).equals(value)) {
                difference.put(key, tempConf.get(key));
            }
        }

        if (difference.size() > 0) {
            Log.info(String.format("Differences with %s:", conf.get("name")), "CONFIG_CHECK");
            for (Map.Entry<String, String> entry : difference.entrySet()) {
                String key = entry.getKey();
                String value = entry.getValue();
                Log.info(String.format("\t'%s': '%s' <> '%s'", key, configurationMap.get(key), value), "CONFIG_CHECK");
            }
        }

        List<String> currentConfiguration = configurationMap.entrySet().stream()
                .map(entry -> entry.getKey() + ": " + entry.getValue())
                .collect(Collectors.toList());

        List<String> pastConfiguration = conf.entrySet().stream()
                .map(entry -> entry.getKey() + ": " + entry.getValue())
                .collect(Collectors.toList());

        Patch<String> patch = DiffUtils.diff(currentConfiguration, pastConfiguration);
        for (Delta<String> delta : patch.getDeltas()) {
            Log.info(delta.toString(), "CONFIG_CHECK");
        }
        return 1 - (double) difference.size() / tempConfMap.size();
    }

    public void trackDatasetDependency(List<Dependency> dependencies) {
        for (Dependency dependency : dependencies) {
            if (dependency.getGroupId().equals("de.embeddedmontiarcdl.datasets") || dependency.getGroupId().equals("de.monticore.lang.monticar.datasets")) {
                String value = String.format("%s:%s:%s", dependency.getGroupId(), dependency.getArtifactId(), dependency.getVersion());
                configurationMap.put("dataset", value);
                break;
            }
        }
    }

    private void createRequiredDirs() {
        File gitlabArtifactsPath = new File(configCheck.getGitlabArtifactsPath());
        File currentExperimentPath = new File(String.format("%s/%s", configCheck.getPathTmp(), configCheck.getExperimentName()));
        if (!gitlabArtifactsPath.exists()) {
            Log.info(String.format("[ConfigCheck] Creating %s", gitlabArtifactsPath.getPath()), "[ConfigCheck]");
            gitlabArtifactsPath.mkdirs();
        }
        if (!currentExperimentPath.exists()) {
            Log.info(String.format("[ConfigCheck] Creating %s", currentExperimentPath.getPath()), "[ConfigCheck]");
            currentExperimentPath.mkdirs();
        }
    }

    private static String extractGitLabAccessTokenFromSettings(Settings settings) {
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
}
