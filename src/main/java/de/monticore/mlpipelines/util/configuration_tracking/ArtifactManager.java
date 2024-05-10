package de.monticore.mlpipelines.util.configuration_tracking;

import com.github.difflib.text.DiffRow;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import conflang._ast.ASTConfLangCompilationUnit;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._ast.ASTArchitecture;
import de.monticore.lang.monticar.cnnarch._ast.ASTLayerDeclaration;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.mlpipelines.automl.emadlprinter.EmadlPrettyPrinter;
import de.monticore.mlpipelines.automl.helper.ArchitectureHelper;
import de.monticore.mlpipelines.tracking.helper.ASTConfLangHelper;
import de.monticore.parsing.ConfigurationLanguageParser;
import de.monticore.parsing.EMADLParser;
import de.monticore.symbolmanagement.SymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;
import org.mlflow.api.proto.Service;
import org.mlflow.tracking.ExperimentsPage;
import org.mlflow.tracking.MlflowClient;
import org.mlflow_project.google.common.reflect.TypeToken;
import java.io.*;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

import static de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab.createSymTab;

public class ArtifactManager {
    private static String model;
    private static String trainAlgorithm;
    private static List<Artifact> gitlabArtifacts;
    private static List<Artifact> mlflowArtifacts;
    private List<Artifact> similarArtifacts;

    // TODO: CHANGE ARTIFACT LOADING TO BE CALLED ONCE AND NOT FOR EACH RUN!!!
    public ArtifactManager(Artifact currentRunArtifact) {
        model = currentRunArtifact.getInfoValue("model");
        trainAlgorithm = currentRunArtifact.getTrainAlgorithmName();
        similarArtifacts = new ArrayList<>();
        gitlabArtifacts = loadGitlabArtifacts();
        mlflowArtifacts = new ArrayList<>();
        List<String> mlflowTrackingUrls = ConfigurationTrackingConf.getMlflowTrackingUrls();
        if (mlflowTrackingUrls != null) {
            for (String mlflowTrackingUrl : mlflowTrackingUrls) {
                try {
                    MlflowClient mlflowClient = new MlflowClient(mlflowTrackingUrl);
                    mlflowArtifacts.addAll(loadMlflowArtifacts(mlflowClient));
                } catch (IllegalArgumentException exception) {
                    Log.error(String.format("Could not load Mlflow artifacts from `%s`", mlflowTrackingUrl));
                }
            }
        }
    }

    public List<Artifact> getSimilarArtifacts() {
        return similarArtifacts;
    }

    public void setSimilarArtifacts(List<Artifact> artifacts) {
        similarArtifacts = artifacts;
    }

    private List<Artifact> loadGitlabArtifacts() {
        List<Artifact> gitlabArtifacts = new ArrayList<>();
        File artifactDir = new File(ConfigurationTrackingConf.getGitlabArtifactsPath());
        for (File experimentDir : Objects.requireNonNull(artifactDir.listFiles())) {
            Set<String> runNames = new TreeSet<>();
            for (File artifactFile : Objects.requireNonNull(experimentDir.listFiles())) {
                if (artifactFile.getName().endsWith(".json")) {
                    runNames.add(artifactFile.getName().split(".json")[0]);
                }
            }
            for (String runName : runNames) {
                Artifact artifact = new Artifact();
                for (File artifactFile : Objects.requireNonNull(experimentDir.listFiles())) {
                    if (!artifactFile.getName().startsWith(runName+".") && !artifactFile.getName().startsWith(runName+"_")) {
                        continue;
                    }
                    if (artifactFile.getName().endsWith(".json")) {
                        try {
                            FileReader reader = new FileReader(artifactFile.getPath());
                            Gson gson = new GsonBuilder().setPrettyPrinting().disableHtmlEscaping().create();
                            Type type = new TypeToken<Map<String, Map<String, String>>>() {}.getType();
                            Map<String, Map<String, String>> artifactMap = gson.fromJson(reader, type);
                            if (artifactMap != null) {
                                artifact.setArtifact(artifactMap);
                            }
                        } catch (FileNotFoundException e) {
                            e.printStackTrace();
                        }
                    } else if (artifactFile.getName().endsWith(".conf")) {
                        String[] parts = artifactFile.getName().replace(".conf", "").split("_");
                        String confName = String.join("_", Arrays.copyOfRange(parts, 1, parts.length));
                        try {
                            artifact.setConfiguration(confName, new ConfigurationLanguageParser().parseModelIfExists(artifactFile.getPath()));
                        } catch (IOException e) {
                            e.printStackTrace();
                        }
                    }
                }
                // Keep only artifacts with the same train algorithm
                if (artifact.getTrainAlgorithmName() == null || artifact.getTrainAlgorithmName().equals(trainAlgorithm)) {
                    gitlabArtifacts.add(artifact);
                }
            }
        }
        Log.info(String.format("Loaded %d experiments with %d artifacts from GitLab", Objects.requireNonNull(artifactDir.listFiles()).length, gitlabArtifacts.size()), "CONFIG_CHECK");
        return gitlabArtifacts;
    }

    private List<Artifact> loadMlflowArtifacts(MlflowClient mlflowClient) {
        ExperimentsPage experimentsPage = mlflowClient.searchExperiments();
        Map<String, String> experimentIdsNames = new HashMap<>();
        for (Service.Experiment experiment : experimentsPage.getItems()) {
            experimentIdsNames.put(experiment.getExperimentId(), experiment.getName());
        }

        // TODO: check this AND CHANGE searchRuns DEPRECATED!!
        StringBuilder searchFilter = new StringBuilder();
        searchFilter.append(String.format("tags.Model ILIKE '%s'", model));
        if (trainAlgorithm != null) {
            searchFilter.append(String.format(" AND params.train_algorithm_name ILIKE '%s'", trainAlgorithm));
        }
//        String searchFilter = String.format("tags.Model ILIKE '%s' and tags.Version='%s'", configurationMap.get("modelToTrain"), configCheck.getVersion());
//        RunsPage mlflowRuns = mlflowClient.searchRuns(new ArrayList<>(experimentIdsNames.keySet()), searchFilter.toString(), Service.ViewType.ALL, 100);
        List<Service.RunInfo> mlflowRuns = mlflowClient.searchRuns(new ArrayList<>(experimentIdsNames.keySet()), searchFilter.toString());
        List<Artifact> mlflowArtifacts_ =  filterMlflowArtifacts(mlflowClient, mlflowRuns, experimentIdsNames);
        Log.info(String.format("Loaded %d artifacts from Mlflow", mlflowArtifacts_.size()), "CONFIG_CHECK");
        return mlflowArtifacts_;
    }

    private List<Artifact> filterMlflowArtifacts(MlflowClient mlflowClient, List<Service.RunInfo> mlflowRuns, Map<String, String> experimentIdsNames) {
        List<Artifact> mlflowArtifacts = new ArrayList<>();
        for (Service.RunInfo runInfo : mlflowRuns) {
            Service.Run run = mlflowClient.getRun(runInfo.getRunId());
            List<Service.Param> runParams = run.getData().getParamsList();

            // Skip the current run
            if (runInfo.getStatus().equals(Service.RunStatus.RUNNING)) {
                continue;
            }

            Artifact mlflowArtifact = new Artifact();
            Map<String, String> info = new TreeMap<>();
            Map<String, String> confTrain = new TreeMap<>();
            info.put("experiment_name", experimentIdsNames.get(runInfo.getExperimentId()));
            info.put("run_name", runInfo.getRunName());
            for (Service.Param param : runParams) {
                confTrain.put(param.getKey(), param.getValue());
            }

            for (Service.RunTag tag : run.getData().getTagsList()) {
                if (tag.getKey().equals("Model")) {
                    info.put("model", tag.getValue());
                } else if (tag.getKey().equals("AutoML Stage")) {
                    info.put("stage", tag.getValue());
                }
            }

            for (Service.Metric metric : run.getData().getMetricsList()) {
                if (metric.getKey().equals("train_accuracy")) {
                    info.put("train_accuracy", String.format("%.2f", metric.getValue()));
                }
            }

            File mlflowArtifactFile = mlflowClient.downloadArtifacts(runInfo.getRunId());
            try {
                String content = new String(Files.readAllBytes(Paths.get(String.format("%s/network.txt", mlflowArtifactFile.getPath()))));
                info.put("network", content);
            } catch (IOException e) {
                Log.error(String.format("Could not load the content of the 'network.txt' artifact for Mlflow run %s: %s", runInfo.getRunId(), e.getMessage()));
                continue;
            }
            mlflowArtifact.put("training_configuration", confTrain);
            mlflowArtifact.put("info", info);
            mlflowArtifacts.add(mlflowArtifact);
        }
        return mlflowArtifacts;
    }

    public boolean hasSimilarArtifacts(Artifact artifact, String artifactType) {
        List<Artifact> artifacts = artifactType.equalsIgnoreCase("gitlab") ? gitlabArtifacts : mlflowArtifacts;
        if (artifacts == null || artifacts.isEmpty()) {
            return false;
        }
        for (Artifact registryArtifact : artifacts) {
            if (artifact.getInfoValue("stage").contains("HO:")) {
                double hoSimilarity = configurationSimilarity(artifact.get("ho_configuration"), registryArtifact.get("ho_configuration"));
                double searchSpaceSimilarity = configurationSimilarity(artifact.get("search_space"), registryArtifact.get("search_space"));
                if (hoSimilarity != 1 || searchSpaceSimilarity != 1) {
                    return false;
                }
            }
            double similarity = artifactSimilarity(artifact, registryArtifact);
            if (similarity == 1) {
                ConfigurationTrackingManager.getArtifact().setAccuracy(registryArtifact.getAccuracy());
                ConfigurationTrackingReport.logDuplicateArtifact(registryArtifact);
                return true;
            }
        }
        return false;
    }

    public static List<Map<String, ASTConfLangCompilationUnit>> getHOConfigurations() {
        List<Map<String, ASTConfLangCompilationUnit>> hoConfigs = new ArrayList<>();
        for (Artifact artifact : gitlabArtifacts) {
            Map<String, ASTConfLangCompilationUnit> hoConf = new HashMap<>();
            hoConf.put("ho_configuration", artifact.getConfiguration("ho_configuration"));
            hoConf.put("search_space", artifact.getConfiguration("search_space"));
            hoConfigs.add(hoConf);
        }
        return hoConfigs;
    }

//    public static void addGitlabArtifact(Artifact artifact) {
//        gitlabArtifacts.add(artifact);
//    }

    private double artifactSimilarity(Artifact artifact1, Artifact artifact2) {
        String[] confNames = new String[]{"training_configuration", "network", "dataset"};
        float similarity = 0;
        for (String confName : confNames) {
            if (confName.equals("network")) {
                similarity += compareNetworks(artifact1.getNetwork(), artifact2.getNetwork());
            } else if (confName.equals("dataset")) {
                Map<String, String> tempMap1 = new TreeMap<>();
                Map<String, String> tempMap2 = new TreeMap<>();
                tempMap1.put(confName, artifact1.getInfoValue(confName));
                tempMap2.put(confName, artifact2.getInfoValue(confName));
                similarity += configurationSimilarity(tempMap1, tempMap2);
            } else {
                similarity += configurationSimilarity(artifact1.get(confName), artifact2.get(confName));
            }
        }
        similarity /= confNames.length;
        artifact2.setSimilarityScore(similarity);
        if (similarity > 0.95) {
            similarArtifacts.add(artifact2);
        }
        return similarity;
    }

    private double configurationSimilarity(Map<String, String> conf1, Map<String, String> conf2) {
        if (conf1 == null || conf2 == null) {
            return 0;
        }

        int diffs_count = 0;
        for (String key : conf1.keySet()) {
            if ((!conf1.containsKey(key) || !conf1.get(key).equals(conf2.get(key)))) {
                diffs_count++;
            }
        }
        return 1 - (double) diffs_count / Math.max(conf1.size(), conf2.size());
    }

    private double compareNetworks(ArchitectureSymbol architectureSymbol1, ArchitectureSymbol architectureSymbol2) {
        List<ArchitectureElementSymbol> elementSymbols1 = getArchitectureElements(architectureSymbol1);
        List<String> layers1 = getLayersFromArchitectureSymbol(elementSymbols1);
        List<ASTLayerDeclaration> methodDeclarations1 = ((ASTArchitecture) architectureSymbol1.getAstNode().get()).getMethodDeclarationList();

        List<ArchitectureElementSymbol> elementSymbols2 = getArchitectureElements(architectureSymbol2);
        List<ASTLayerDeclaration> methodDeclarations2 = ((ASTArchitecture) architectureSymbol2.getAstNode().get()).getMethodDeclarationList();
        List<String> layers2 = getLayersFromArchitectureSymbol(elementSymbols2);

        if (!layers1.equals(layers2)) {
            return 0;
        }
        if (methodDeclarations1.size() == methodDeclarations2.size()) {
            for (int i = 0; i < methodDeclarations1.size(); i++) {
                if (!methodDeclarations1.get(i).deepEquals(methodDeclarations2.get(i))) {
                    return 0;
                }
            }
        }
        return 1;
    }

    private List<String> getLayersFromArchitectureSymbol(List<ArchitectureElementSymbol> architectureElementSymbol) {
        List<String> layers = new ArrayList<>();
        for (ArchitectureElementSymbol symbol : architectureElementSymbol) {
            try{
                List<MutableScope> scopes = symbol.getSpannedScope().getSubScopes();
                for (MutableScope scope : scopes) {
                    layers.add(scope.getName().get());
                }
            } catch (Exception e) {
                layers.add(symbol.getName());
            }
        }
        return layers;
    }

    private List<ArchitectureElementSymbol> getArchitectureElements(ArchitectureSymbol architectureSymbol) {
        List<ArchitectureElementSymbol> elementSymbols = new ArrayList<>();
        for (NetworkInstructionSymbol instructionSymbol : architectureSymbol.getNetworkInstructions()) {
            elementSymbols.addAll(instructionSymbol.getBody().getElements());
        }
        return elementSymbols;
    }

//    private static void saveNetworkIntoFile(String network, String networkName) {
////        String modelPath = "src/main/resources/efficientnet_experiment/emadl";
//        String tempName = "temp.emadl";
//        File adaNetCustomFile = new File(String.format("%s/%s/%s.emadl", ConfigurationTrackingConf.getModelPath(), ConfigurationTrackingConf.getModelName().split("\\.")[0], networkName));
////        File adaNetCustomFile = new File(String.format("%s/%s/%s.emadl", modelPath, "mnist", networkName));
//        if (adaNetCustomFile.exists()) {
//            File tempFile = new File(String.format("%s/%s", adaNetCustomFile.getParentFile().getPath(), tempName));
//            boolean renamed = adaNetCustomFile.renameTo(tempFile);
//            if (renamed) {
//                try {
//                    FileWriter networkFileWriter = new FileWriter(String.format("%s/%s.emadl", adaNetCustomFile.getParentFile().getPath(), networkName));
//                    networkFileWriter.write(network);
//                    networkFileWriter.close();
//                } catch (IOException e) {
//                    System.out.println("saveNetworkIntoFile: " + e.getMessage());
//                }
//            } else {
//                System.out.println("renaming failed");
//            }
//        } else {
//            System.out.printf("couldn't find %s.emadl\n", networkName);
//        }
//    }
//
//    private static void resetNetworkFile(String networkName) {
//        String tempName = "temp.emadl";
////        String modelPath = "src/main/resources/efficientnet_experiment/emadl";
////        File adaNetCustomFile = new File(String.format("%s/%s/%s.emadl", modelPath, "mnist", networkName));
//        File adaNetCustomFile = new File(String.format("%s/%s/%s.emadl", ConfigurationTrackingConf.getModelPath(), ConfigurationTrackingConf.getModelName().split("\\.")[0], networkName));
//        File tempFile = new File(String.format("%s/%s/%s", ConfigurationTrackingConf.getModelPath(), ConfigurationTrackingConf.getModelName().split("\\.")[0], tempName));
//        if (tempFile.exists()) {
//            tempFile.renameTo(new File(String.format("%s/%s.emadl", adaNetCustomFile.getParentFile().getPath(), networkName)));
//        }
//    }
//
//    private static ArchitectureSymbol resolveNetwork(String networkName) {
////        String modelPath = "src/main/resources/efficientnet_experiment/emadl";
//        String rootComponentPath = Paths.get(ConfigurationTrackingConf.getModelPath(), model.replaceFirst("\\.", "/") + ".emadl").toString();
////        String rootComponentPath = Paths.get(modelPath, "mnist/MnistClassifier.emadl").toString();
//        ModelPath modelPath_ = new ModelPath(Paths.get(ConfigurationTrackingConf.getModelPath()));
////        ModelPath modelPath_ = new ModelPath(Paths.get(modelPath));
//        ASTEMACompilationUnit rootComponent = null;
//        try {
//            rootComponent = new EMADLParser().parseModelIfExists(rootComponentPath);
//        } catch (IOException e) {
//            System.out.println("resolveNetwork: " + rootComponentPath + ": " + e.getMessage());
//        }
//
//        if (rootComponent != null) {
//            Scope emadlSymbolTable = SymbolTableCreator.createEMADLSymbolTable(rootComponent, new GlobalScope(modelPath_, new EMADLLanguage()));
//            String rootComponentName = rootComponent.getComponent().getName();
//            String instanceName = rootComponentName.substring(0, 1).toLowerCase() + rootComponentName.substring(1);
//            EMAComponentInstanceSymbol componentInstance = (EMAComponentInstanceSymbol) emadlSymbolTable.resolve(
//                    instanceName, EMAComponentInstanceSymbol.KIND).get();
//            for (EMAComponentInstanceSymbol subcomponent : componentInstance.getSubComponents()) {
//                EMAComponentSymbol referencedSymbol = subcomponent.getComponentType().getReferencedSymbol();
//                Optional<ArchitectureSymbol> network_ = referencedSymbol.getSpannedScope()
//                        .resolve("", ArchitectureSymbol.KIND);
//                if (network_.isPresent() && network_.get().getEnclosingScope().getEnclosingScope().get().getName().get().equals(networkName)) {
//                    return network_.get();
//                }
//            }
//        }
//        return null;
//    }
//
//    private static ArchitectureSymbol parseArchitectureSymbol_String(String network) {
//        String networkName = StringUtils.capitalize(network.split("<")[0].split("component ")[1].trim());
//        saveNetworkIntoFile(network,networkName);
//        ArchitectureSymbol architectureSymbol = resolveNetwork(networkName);
//        resetNetworkFile(networkName);
//        return architectureSymbol;
//    }

//    public static void main(String[] args) {
//        String networkStr = "package mnist;\n\ncomponent EfficientNetBase<Z(2:oo) classes = 10, Z(1:oo) layerWidth = 10>{\n" +
//                "    ports in Z(0:255)^{1, 28, 28} image, out Q(0:1)^{classes} predictions;\n" +
//                "\n" +
//                "    implementation CNN {\n" +
//                "        def stem(channels){\n" +
//                "            Convolution(kernel = (3,3), channels = channels, stride = (2,2), padding = \"same\") ->\n" +
//                "            BatchNorm() ->\n" +
//                "            Relu()\n" +
//                "        }\n" +
//                "\n" +
//                "def stem(channels){\n" +
//            "            Convolution(kernel = (3,3), channels = channels, stride = (2,2), padding = \"same\") ->\n" +
//                    "            BatchNorm() ->\n" +
//                    "            Relu()\n" +
//                    "        }\n" +
//                    "\n" +
//                "        def conv(channels){\n" +
//                "            Convolution(kernel = (3,3), channels = channels, stride = (1,1), padding = \"same\") ->\n" +
//                "            BatchNorm() ->\n" +
//                "            Relu()\n" +
//                "        }\n" +
//                "\n" +
//                "        def reductionConv(channels){\n" +
//                "            Convolution(kernel = (3,3), channels = channels, stride = (2,2), padding = \"same\") ->\n" +
//                "            BatchNorm() ->\n" +
//                "            Relu()\n" +
//                "        }\n" +
//                "\n" +
//                "        def residualBlock(channels){\n" +
//                "            conv(channels = channels) ->\n" +
//                "            conv(channels = channels) ->\n" +
//                "            Relu()\n" +
//                "        }\n" +
//                "\n" +
//                "        def reductionBlock(channels){\n" +
//                "            (\n" +
//                "                conv(channels = channels) ->\n" +
//                "                conv(channels = channels) ->\n" +
//                "                reductionConv(channels = channels)\n" +
//                "            |\n" +
//                "                reductionConv(channels = channels)\n" +
//                "            ) ->\n" +
//                "            Add()\n" +
//                "        }\n" +
//                "\n" +
//                "        image ->\n" +
//                "        stem(channels = 4) ->\n" +
//                "        residualBlock(-> = 4, channels = 16) ->\n" +
//                "        reductionBlock(channels = 16) ->\n" +
//                "        residualBlock(-> = 4, channels = 32) ->\n" +
//                "        reductionBlock(channels = 32) ->\n" +
//                "        FullyConnected(units = classes) ->\n" +
//                "        Softmax() ->\n" +
//                "        predictions\n" +
//                "        ;\n" +
//                "    }\n" +
//                "}";
//        ArchitectureSymbol architectureSymbol = parseArchitectureSymbol_String(networkStr);
//        System.out.println(new EmadlPrettyPrinter().prettyPrint(architectureSymbol));
//    }
}
