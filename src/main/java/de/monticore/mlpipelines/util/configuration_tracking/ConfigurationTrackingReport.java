package de.monticore.mlpipelines.util.configuration_tracking;

import com.github.difflib.text.DiffRow;
import com.github.difflib.text.DiffRowGenerator;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.mlpipelines.automl.emadlprinter.ASTConfLangCompilationUnitPrinter;
import de.monticore.mlpipelines.automl.emadlprinter.EmadlPrettyPrinter;
import de.monticore.mlpipelines.pipelines.Pipeline;
import de.se_rwth.commons.logging.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.stream.Collectors;

public class ConfigurationTrackingReport {
    private static final StringBuilder report = new StringBuilder();
    private static final String reportFilePath = String.format("%s/report.txt", ConfigurationTrackingConf.getPathTmp());

    public static void logMessage(String message) {
        report.append("\t").append(message).append("\n");
    }

    public static void logDuplicateArtifact(Artifact artifact) {
        report.append(String.format("\t'%s' from the '%s' experiment has the same configuration and achieved train accuracy %.2f\n", artifact.getInfoValue("run_name"), artifact.getExperimentName(), artifact.getAccuracy()));
    }

    public static void logInitializeCheck(Artifact artifact) {
        report.append("-------------------------------------------------------------------------------------------------").append("\n");
        report.append("\n").append(String.format("Running `%s` from the current experiment (stage = %s):", artifact.getInfoValue("run_name"), artifact.getInfoValue("stage"))).append("\n");
    }

    public static void logPipeline(Pipeline pipeline) {
        float accuracy = ConfigurationTrackingManager.getArtifact().getAccuracy();
        logMessage("The model was successfully trained with the following training configuration:\n");
        logMessage(new ASTConfLangCompilationUnitPrinter().prettyPrint(pipeline.getTrainingConfiguration()));
        logMessage("and the following network architecture:\n");
        logMessage(new EmadlPrettyPrinter().prettyPrint((ArchitectureSymbol) pipeline.getNeuralNetwork().getSpannedScope().getSubScopes().get(0).getSpanningSymbol().get()));
        logMessage(String.format("The model achieved train accuracy: %.2f", accuracy));
    }

    public static void saveReport() {
        try {
            FileWriter reportFile = new FileWriter(reportFilePath);
            reportFile.write(report.toString());
            reportFile.close();
        } catch (IOException e) {
            Log.info("Could not save the report into " + reportFilePath, ConfigurationTrackingReport.class.getName());
        }

    }
//    public static void printSummaryForArtifacts(Artifact artifact1, Artifact artifact2) {
//        StringBuilder summary = new StringBuilder();
//        summary.append(String.format("Diff summary with run `%s`\n\n", artifact2.getArtifactName()))
//                .append(String.format("* similarity: %.2f\n\n", artifact2.getSimilarityScore()))
//                .append("* accuracy:\n")
//                .append(String.format("\tThe trained accuracy for the run `%s` is `%s`\n\n", artifact2.getArtifactName(), artifact2.getAccuracy()))
//                .append(generateDatasetSummary(artifact1, artifact2))
//                .append(generateNetworkSummary(artifact1, artifact2));
//
//        List<String> artifactKeys = Arrays.asList("nas_configuration", "ho_configuration", "training_configuration", "evaluation_criteria");
//        for (String key : artifactKeys) {
//            if (artifact1.get(key) == null || artifact2.get(key) == null) {
//                summary.append(String.format("Missing %s configuration for one of the runs\n\n", key));
//                continue;
//            }
//            summary.append(generateConfigurationSummary(artifact1, artifact2, key)).append("\n");
//        }
//
//        File runDir = new File(String.format("%s/%s", ConfigurationTrackingConf.getArtifactsDiffPath(), artifact1.getInfoValue("run_name")));
//        runDir.mkdirs();
//        String filePath = String.format("%s/%s.diff.txt", runDir.getPath(), artifact2.getArtifactName());
//        writeSummaryToFile(summary.toString(), filePath);
//    }
//
//    private static void writeSummaryToFile(String summary, String filePath) {
//        try {
//            BufferedWriter writer = new BufferedWriter(new FileWriter(filePath, true));
//            writer.write(summary);
//            writer.newLine();
//            writer.newLine();
//            writer.close();
//        } catch (Exception e) {
//            Log.info(String.format("Could not write summary into %s: %s", filePath, e.getMessage()), "CONFIG_CHECK");;
//        }
//    }
//
//    private static String generateNetworkSummary(Artifact artifact1, Artifact artifact2) {
//        StringBuilder networkSummary = new StringBuilder();
//        networkSummary.append("* network:\n");
//        String network1 = artifact1.getInfoValue("network");
//        String network2 = artifact2.getInfoValue("network");
//        if (network1 != null && network1.equals(network2)) {
//            networkSummary.append(String.format("\tThe same network configuration has been used for both runs\n`%s`\n\n", network1));
//        } else if (network1 != null && network2 != null) {
//            List<DiffRow> diffRows = generateDiffRows(Arrays.asList(network1.split("\n")), Arrays.asList(network2.split("\n")));
//            networkSummary.append(String.format("|%-90s|%-90s|\n", "Current run", artifact2.getArtifactName()));
//            for (DiffRow row : diffRows) {
//                String line = String.format("|%-90s|%-90s|\n", row.getOldLine(), row.getNewLine());
//                networkSummary.append(line);
//            }
//        }
//        return networkSummary.toString();
//    }
//
//    private static String generateDatasetSummary(Artifact artifact1, Artifact artifact2) {
//        StringBuilder datasetSummary = new StringBuilder();
//        datasetSummary.append("* dataset:\n");
//        String dataset1 = artifact1.getInfoValue("dataset");
//        String dataset2 = artifact2.getInfoValue("dataset");
//        if (dataset1 != null && dataset1.equals(dataset2)) {
//            datasetSummary.append(String.format("\tThe same dataset has been used for both runs `%s`\n\n", dataset1));
//        } else {
//            datasetSummary.append(String.format("\tDifferent datasets have been used for the runs `%s` and `%s`\n\n", dataset1, dataset2));
//        }
//        return datasetSummary.toString();
//    }
//
//    private static String generateConfigurationSummary(Artifact artifact1, Artifact artifact2, String configurationKey) {
//        StringBuilder configurationSummary = new StringBuilder();
//        configurationSummary.append(String.format("* %s:\n", configurationKey));
//        List<DiffRow> diffRows = generateDiffRows(getLinesFromMap(artifact1.get(configurationKey)), getLinesFromMap(artifact2.get(configurationKey)));
//        configurationSummary.append(String.format("|%-90s|%-90s|\n", "Current run", artifact2.getArtifactName()));
//        for (DiffRow row : diffRows) {
//            String line = String.format("|%-90s|%-90s|\n", row.getOldLine(), row.getNewLine());
//            configurationSummary.append(line);
//        }
//        String trainAlgName1 = artifact1.getTrainAlgorithmName();
//        String trainAlgName2 = artifact2.getTrainAlgorithmName();
//        if (trainAlgName1 != null && trainAlgName2 != null && !trainAlgName1.equals(trainAlgName2)) {
//            configurationSummary.append("Different training algorithm has been used for the both runs.\n\n");
//        } else {
//            configurationSummary.append(generateSummaryFromDiffRows(diffRows));
//        }
//        return configurationSummary.toString();
//    }
//
//    private static String generateSummaryFromDiffRows(List<DiffRow> diffRows) {
//        String[] diffChars = new String[]{"*", "~"};
//        boolean hasDiff = false;
//        StringBuilder confSummary = new StringBuilder();
//
//        if (diffRows.isEmpty()) {
//            confSummary.append("\tNo diffs found\n");
//            return confSummary.toString();
//        }
//
//        for (DiffRow row : diffRows) {
//            if (Arrays.stream(diffChars).anyMatch(row.getOldLine()::contains) || Arrays.stream(diffChars).anyMatch(row.getNewLine()::contains)) {
//                if (!hasDiff) {
//                    confSummary.append("\tThe differences between the configurations are the following:\n");
//                }
//
//                String[] oldLineParts = row.getOldLine().replaceAll("[~*]", "").replace("-&gt;", "").split(": ");
//                String[] newLineParts = row.getNewLine().replaceAll("[~*]", "").replace("-&gt;", "").split(": ");
//                if (oldLineParts.length == newLineParts.length && oldLineParts[0].equals(newLineParts[0])) {
//                    confSummary.append(String.format("\t\tThe `%s` value has been changed from `%s` to `%s`;\n", oldLineParts[0], oldLineParts[1], newLineParts[1]));
//                } else {
//                    String keyOld = oldLineParts[0];
//                    String keyNew = newLineParts[0];
//                    if (keyOld.isEmpty() || keyNew.isEmpty()) {
//                        confSummary.append(String.format("\t\tThe key `%s` does not exist in both configurations\n", keyOld.isEmpty() ? keyNew : keyOld));
//                    } else {
//                        confSummary.append(String.format("\t\tCompared 2 different keys `%s` and `%s`. The train algorithm for both configurations might be different\n", oldLineParts[0], newLineParts[0]));
//                    }
//                }
//                hasDiff = true;
//            }
//        }
//
//        if (!hasDiff) {
//            confSummary.append("\tThe configurations are equal\n");
//        }
//        return confSummary.toString();
//    }
//
//    public static List<DiffRow> generateDiffRows(List<String> pastConfiguration, List<String> currentConfiguration) {
//        if (pastConfiguration == null || pastConfiguration.isEmpty() || currentConfiguration == null || currentConfiguration.isEmpty()) {
//            return new ArrayList<>();
//        }
//        DiffRowGenerator generator = DiffRowGenerator.create()
//                .showInlineDiffs(true)
//                .inlineDiffByWord(true)
//                .oldTag(f -> "~")
//                .newTag(f -> "**")
//                .build();
//        return generator.generateDiffRows(pastConfiguration, currentConfiguration);
//    }
//
//    private static List<String> getLinesFromMap(Map<String, String> map) {
//        return map.entrySet().stream()
//                .map(entry -> String.format("%s: %s", entry.getKey(), entry.getValue()))
//                .collect(Collectors.toList());
//    }
}
