package de.monticore.mlpipelines.util.configuration_tracking;


import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import conflang._ast.ASTConfLangCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.mlpipelines.automl.emadlprinter.EmadlPrettyPrinter;
import de.monticore.mlpipelines.automl.helper.ConfigurationValidationHandler;
import de.monticore.mlpipelines.configuration.MontiAnnaContext;
import de.monticore.mlpipelines.pipelines.Pipeline;
import de.se_rwth.commons.logging.Log;

import java.io.*;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.*;
import java.util.stream.Collectors;

import static de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab.createSymTab;

public class ConfigurationTrackingManager {
    private static int runId = 0;
    private static ArtifactManager artifactManager;
    private static Artifact artifact;
    private static boolean deployArtifact = false;


    public static void applyConfiguration(Pipeline trainPipeline, MontiAnnaContext montiAnnaContext) {
        ConfigurationTrackingConf.setModelName(montiAnnaContext.getRootModelName());
        ConfigurationTrackingConf.setModelPath(montiAnnaContext.getParentModelPath().toString());
        artifact = new Artifact();
        artifact.setConfiguration("training_configuration", trainPipeline.getTrainingConfiguration());
        artifact.setConfigurationEntry("info", "model", montiAnnaContext.getRootModelName());
        artifact.setConfigurationEntry("info", "experiment_name", ConfigurationTrackingConf.getExperimentName());
        artifact.setConfigurationEntry("info", "network_name", trainPipeline.getNeuralNetwork().getComponentType().getName());
        loadDatasetInfo();
        artifactManager = new ArtifactManager(artifact);
    }

    public static void applyConfiguration(ASTConfLangCompilationUnit nasConf, ASTConfLangCompilationUnit searchSpace, ASTConfLangCompilationUnit hyperparamsOptConf, ASTConfLangCompilationUnit evaluationCriteria, Pipeline trainPipeline, MontiAnnaContext montiAnnaContext) {
        applyConfiguration(trainPipeline, montiAnnaContext);
        artifact.setConfiguration("nas_configuration", nasConf);
        artifact.setConfiguration("ho_configuration", hyperparamsOptConf);
        artifact.setConfiguration("evaluation_criteria", evaluationCriteria);
        artifact.setConfiguration("search_space", searchSpace);
        artifactManager = new ArtifactManager(artifact);
    }

    public static void loadDatasetInfo() {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(String.format("%s/%s", ConfigurationTrackingConf.getPathTmp(), ConfigurationTrackingConf.getDatasetFileName())));
            String dataset = reader.readLine();
            artifact.setConfigurationEntry("info", "dataset", dataset);
        } catch (IOException e) {
            Log.info(String.format("The dataset info could not be loaded from %s. Error: %s", ConfigurationTrackingConf.getPathTmp(), e.getMessage()), "CONFIGURATION_TRACKING");
        }
    }

    public static void setArtifactConfigurationEntry(String configurationKey, String key, String value) {
        artifact.setConfigurationEntry(configurationKey, key, value);
    }

    public static int getRunId() {
        return runId;
    }

    public static Artifact getArtifact() {
        return artifact;
    }

    public static void setArtifact(Artifact artifact1) {
        artifact = artifact1;
    }

    private static String getTargetPath(String pathTmp, String experimentName) {
        if (experimentName == null || experimentName.isEmpty()) {
            String uuidRegex = "[a-f0-9]{8}-([a-f0-9]{4}){2}-[a-f0-9]{12}";
            for (File file : Objects.requireNonNull(new File(pathTmp).listFiles())) {
                if (file.isDirectory() && file.getName().matches(uuidRegex)) {
                    return file.getPath();
                }
            }
        }
        return String.format("%s/%s", pathTmp, experimentName);
    }

    public static void initializeNewRun(Pipeline pipeline, String stage) {
        artifact.setConfiguration("training_configuration", pipeline.getPipelineConfiguration());
        artifact.setConfigurationEntry("info", "run_name", String.format("Run %d", ++runId));
        artifact.setConfigurationEntry("info", "stage", stage);
        artifact.setNetworkString(getNetworkString(pipeline.getNeuralNetwork()));
        artifact.setConfigurationEntry("info", "train_accuracy", "0");
        artifactManager.setSimilarArtifacts(new ArrayList<>());
    }

    public static void saveRun(Pipeline trainPipeline) {
        if (!ConfigurationTrackingConf.isEnabled()) {
            return;
        }
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("dd.MM.yyyy HH:mm:ss");
        artifact.setConfigurationEntry("info", "run_at", LocalDateTime.now().format(formatter));
        artifact.setConfiguration("training_configuration", trainPipeline.getTrainingConfiguration());
        String targetPath = getTargetPath(ConfigurationTrackingConf.getPathTmp(), ConfigurationTrackingConf.getExperimentName());
        artifact.saveArtifact(targetPath);
    }

    public static void executePipeline(Pipeline pipeline, String stage) {
        if (!ConfigurationTrackingConf.isEnabled()) {
            pipeline.execute();
            return;
        }

        initializeNewRun(pipeline, stage);
//        if (artifact.getInfoValue("stage").contains("HO:")) {
//            List<Map<String, ASTConfLangCompilationUnit>> hoSetups = ArtifactManager.getHOConfigurations();
//
//            ConfigurationValidationHandler.validateConfiguration(trainingConfiguration, pipeline.getSchemasTargetDir());
//            pipeline.setTrainingConfiguration(trainingConfiguration);
//            // Utilize "ho_configuration", "search_space" in order to find better hyperparameter values
//        if (!artifact1.getInfoValue("stage").contains("HO:") && (confName.equals("ho_configuration") || confName.equals("search_space"))) {
//            similarity += 1;
//        }
//        }

        if (configurationNotRun()) {
            pipeline.execute();
            artifact.setAccuracy(pipeline.getTrainedAccuracy());
            ConfigurationTrackingReport.logPipeline(pipeline);
        }
        saveRun(pipeline);
        if (stage.equals("Final Run") || stage.equals("MontiAnna Run")) {
            ConfigurationTrackingReport.saveReport();
        }
    }

    public static boolean configurationNotRun() {
        ConfigurationTrackingReport.logInitializeCheck(artifact);
        if (artifactManager.hasSimilarArtifacts(artifact, "GitLab") || artifactManager.hasSimilarArtifacts(artifact, "Mlflow")) {
            return false;
        }
        deployArtifact = true;
        if (artifact.getInfoValue("stage").contains("HO:")) {
            proposeConfigurations();
        }
        ConfigurationTrackingReport.logMessage("No similar artifacts were found, training model...");
        return true;
    }

    public static boolean shouldDeployArtifact() {
        return deployArtifact;
    }

    private static void proposeConfigurations() {
        List<Artifact> similarArtifacts = artifactManager.getSimilarArtifacts().stream()
                .filter(artifact1 -> !artifact.get("ho_configuration").equals(artifact1.get("ho_configuration")))
                .sorted((a1, a2) -> Float.compare(a2.getAccuracy(), a1.getAccuracy()))
                .collect(Collectors.toList());
        if (!similarArtifacts.isEmpty()) {

        }
    }
//
//    private static void analyzeSignificantParamValues(List<Artifact> artifacts, String significantParamName) {
//        String currentParamValue = artifact.getParameterValue(significantParamName);
//        Set<String> significantParamValues = getSignificantParamValues(artifacts).get(significantParamName);
//        if (significantParamValues == null) {
//            return;
//        }
//        for (String paramValue : significantParamValues) {
//            if (paramValue.equals(currentParamValue)) {
//                continue;
//            }
//            List<Artifact> similarParamArtifacts = artifacts.stream()
//                    .filter(artifact1 -> artifact1.getParameterValue(significantParamName).equals(paramValue))
//                    .sorted(Comparator.comparingDouble(Artifact::getSimilarityScore).reversed().thenComparing(Artifact::getAccuracy, Comparator.reverseOrder()))
//                    .collect(Collectors.toList());
//            List<Artifact> similarArtifacts;
//            if (similarParamArtifacts.size() > 3) {
//                similarArtifacts = similarParamArtifacts.subList(0, 3);
//            } else {
//                similarArtifacts = similarParamArtifacts;
//            }
//            if (similarArtifacts.size() > 0) {
//                Log.info(String.format("\t\t%s = %s", significantParamName, paramValue), "CONFIG_CHECK");
//                for (Artifact artifact1 : similarArtifacts) {
//                    Log.info(String.format("\t\t\t[%.2f]`%s` with the parameter `%s` set to `%s` achieved accuracy %.2f", artifact1.getSimilarityScore(), artifact1.getArtifactName(), significantParamName, paramValue, artifact1.getAccuracy()), "CONFIG_CHECK");
//                }
//            }
//        }
//    }
//
//    private static Map<String, Set<String>> getSignificantParamValues(List<Artifact> similarArtifacts) {
//        Map<String, Set<String>> paramValuesMap = new HashMap<>();
//        for (String paramName : ConfigurationTrackingConf.getSignificantParams()) {
//            Set<String> paramValues = similarArtifacts.stream()
//                    .filter(similarArtifact -> similarArtifact.getParameterValue(paramName) != null)
//                    .map(similarArtifact -> similarArtifact.getParameterValue(paramName))
//                    .collect(Collectors.toSet());
//            if (paramValues.size() > 0) {
//                paramValuesMap.computeIfAbsent(paramName, k -> new HashSet<>());
//                paramValuesMap.get(paramName).addAll(paramValues);
//            }
//        }
//        return paramValuesMap;
//    }

    private static String getNetworkString(EMAComponentInstanceSymbol componentInstanceSymbol) {
        ArchitectureSymbol architectureSymbol = (ArchitectureSymbol) componentInstanceSymbol.getSpannedScope().getSubScopes().get(0).getSpanningSymbol().get();
        String networkStr = new EmadlPrettyPrinter().prettyPrint(architectureSymbol)
                .replaceAll("component.*<", String.format("component %s<", artifact.getInfoValue("network_name")))
                .replace(", ;", ";");
        return String.format("package %s;\n\n%s", componentInstanceSymbol.getPackageName().split("\\.")[0], networkStr);
    }
}
