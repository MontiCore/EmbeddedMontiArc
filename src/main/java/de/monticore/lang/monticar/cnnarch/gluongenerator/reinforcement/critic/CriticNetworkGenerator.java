package de.monticore.lang.monticar.cnnarch.gluongenerator.reinforcement.critic;

import com.google.common.collect.Lists;
import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2Gluon;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2GluonArchitectureSupportChecker;
import de.monticore.lang.monticar.cnnarch.gluongenerator.CNNArch2GluonLayerSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.CNNArchSymbolCompiler;
import de.monticore.lang.monticar.cnnarch.generator.TemplateConfiguration;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.cnntrain.annotations.Range;
import de.monticore.lang.monticar.cnntrain.annotations.TrainedArchitecture;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.lang3.StringUtils;

import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

import static com.google.common.base.Preconditions.checkState;

public class CriticNetworkGenerator {
    private static final String START_SEQUENCE = "implementationCritic(state,action)";

    private String generationTargetPath;
    private String rootModelsDir;

    protected String getGenerationTargetPath() {
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String getGenerationTargetPath) {
        this.generationTargetPath = getGenerationTargetPath;
    }

    protected String getRootModelsDir() {
        return rootModelsDir;
    }

    public void setRootModelsDir(String rootModelsDir) {
        this.rootModelsDir = rootModelsDir;
    }

    public CriticNetworkGenerationPair generateCriticNetworkContent(TemplateConfiguration templateConfiguration,
                                                            ConfigurationSymbol configurationSymbol) {
        checkState(getRootModelsDir() != null, "Root project directory is not set");
        checkState(getGenerationTargetPath() != null, "Target path is not set");

        failIfArchitectureNotAvailable(configurationSymbol);
        assert configurationSymbol.getTrainedArchitecture().isPresent();
        TrainedArchitecture trainedArchitecture = configurationSymbol.getTrainedArchitecture().get();
        failIfActorHasMultipleIO(trainedArchitecture);

        List<String> criticNetwork = retrieveFullNameOfCriticsNetworkFromConfiguration(configurationSymbol);
        final String criticNetworkName = criticNetwork.get(criticNetwork.size()-1);
        final Path pathTocriticNetworkFile = retrievePathToCriticNetworkFileFromFullName(criticNetwork, criticNetworkName);
        final String networkImplementation = parseNetworkImplementationFromFile(pathTocriticNetworkFile);

        Map<String, Object> context = makeTemplateContextMap(trainedArchitecture, criticNetworkName, networkImplementation);
        Path directoryOfCnnArchFile = makeCnnArchFileFromContext(templateConfiguration, context);

        Map<String, String> fileContentMap = generatePythonNetworkFiles(criticNetworkName, directoryOfCnnArchFile);
        deleteOutputDirectory(directoryOfCnnArchFile);

        return new CriticNetworkGenerationPair(criticNetworkName, fileContentMap);
    }

    private void deleteOutputDirectory(Path directoryOfCnnArchFile) {
        try {
            FileUtils.deleteDirectory(directoryOfCnnArchFile.toFile());
        } catch (IOException e) {
            Log.warn("Cannot delete temporary CNN arch directory: " + directoryOfCnnArchFile.toString());
        }
    }

    private Map<String, String> generatePythonNetworkFiles(String criticNetworkName, Path directoryOfCnnArchFile) {
        CNNArch2Gluon gluonGenerator = new CNNArch2Gluon();
        gluonGenerator.setGenerationTargetPath(this.getGenerationTargetPath());

        Map<String, String> fileContentMap = new HashMap<>();
        CNNArchSymbolCompiler symbolCompiler = new CNNArchSymbolCompiler(new CNNArch2GluonArchitectureSupportChecker(),
                                                                         new CNNArch2GluonLayerSupportChecker());
        ArchitectureSymbol architectureSymbol = symbolCompiler.compileArchitectureSymbolFromModelsDir(directoryOfCnnArchFile, criticNetworkName);
        architectureSymbol.setComponentName(criticNetworkName);
        fileContentMap.putAll(gluonGenerator.generateStringsAllowMultipleIO(architectureSymbol, true));
        return fileContentMap;
    }

    private Path makeCnnArchFileFromContext(TemplateConfiguration templateConfiguration, Map<String, Object> context) {
        final String architectureContent = templateConfiguration.processTemplate(
            context, "reinforcement/architecture/CriticArchitecture.ftl");

        Path tmpDirectoryToArchFile = Paths.get(this.getGenerationTargetPath(), "tmp");
        try {
            if (!tmpDirectoryToArchFile.toFile().exists()) {
                Files.createDirectories(tmpDirectoryToArchFile);
            }
            Files.write(
                Paths.get(tmpDirectoryToArchFile.toString(), context.get("architectureName") + ".cnna"), architectureContent.getBytes());
        } catch (IOException e) {
            failWithMessage(e.getMessage());
        }
        return tmpDirectoryToArchFile;
    }

    private Map<String, Object> makeTemplateContextMap(TrainedArchitecture trainedArchitecture, String criticNetworkName, String networkImplementation) {
        final String stateName = trainedArchitecture.getInputs().get(0);
        final String actionName = trainedArchitecture.getOutputs().get(0);

        Map<String, List<Integer>> dimensions = trainedArchitecture.getDimensions();
        List<Integer> stateDimensions = dimensions.get(stateName);
        List<Integer> actionDimensions = dimensions.get(actionName);

        Map<String, Range> ranges = trainedArchitecture.getRanges();
        Range stateRange = ranges.get(stateName);
        Range actionRange = ranges.get(actionName);

        Map<String, String> types = trainedArchitecture.getTypes();
        final String stateType = types.get(stateName);
        final String actionType = types.get(actionName);

        Map<String, Object> context = new HashMap<>();
        context.put("stateDimension", stateDimensions);
        context.put("actionDimension", actionDimensions);
        context.put("stateRange", stateRange);
        context.put("actionRange", actionRange);
        context.put("stateType", stateType);
        context.put("actionType", actionType);
        context.put("implementation", networkImplementation);
        context.put("architectureName", criticNetworkName);
        return context;
    }

    private String parseNetworkImplementationFromFile(Path criticNetworkFile) {
        String criticNetworkFileContent = null;
        try {
            criticNetworkFileContent = new String(Files.readAllBytes(criticNetworkFile), Charset.forName("UTF-8"));
        } catch (IOException e) {
            failWithMessage("Cannot create critic network file:" + e.getMessage());
        }

        String contentWhiteSpaceRemoved = criticNetworkFileContent.replaceAll("\\s+","");

        if (!contentWhiteSpaceRemoved.contains(START_SEQUENCE)
            || StringUtils.countMatches(contentWhiteSpaceRemoved, "{") != 1
            || StringUtils.countMatches(contentWhiteSpaceRemoved, "}") != 1
            || contentWhiteSpaceRemoved.charAt(contentWhiteSpaceRemoved.length() - 1) != '}') {
            failWithMessage("Cannot parse critic file");
        }

        final int startOfNNImplementation = contentWhiteSpaceRemoved.indexOf("{") + 1;
        final int endOfNNImplementation = contentWhiteSpaceRemoved.indexOf("}");
        return contentWhiteSpaceRemoved.substring(startOfNNImplementation, endOfNNImplementation);
    }

    private Path retrievePathToCriticNetworkFileFromFullName(List<String> criticNetwork, String criticNetworkName) {
        // Add file ending cnna to file name
        criticNetwork.set(criticNetwork.size()-1, criticNetworkName + ".cnna");

        Path root = Paths.get(this.getRootModelsDir());
        Path criticNetworkFile = criticNetwork.stream().map(Paths::get).reduce(root, Path::resolve);

        if (!criticNetworkFile.toFile().exists()) {
            failWithMessage("Critic network file does not exist in " + criticNetworkFile.toString());
        }
        return criticNetworkFile;
    }

    private List<String> retrieveFullNameOfCriticsNetworkFromConfiguration(ConfigurationSymbol configurationSymbol) {
        // Load critics network
        failIfConfigurationHasNoCritic(configurationSymbol);

        assert configurationSymbol.getEntry("critic").getValue().getValue() instanceof String;
        List<String> criticNetwork = Lists.newArrayList((
            (String)configurationSymbol.getEntry("critic").getValue().getValue()).split("\\."));

        // Check if file name is upper case otherwise make it upper case
        int size = criticNetwork.size();
        if (Character.isLowerCase(criticNetwork.get(size-1).charAt(0))) {
            String lowerCaseFileName = criticNetwork.get(size-1);
            String upperCaseFileName = lowerCaseFileName.substring(0,1).toUpperCase() + lowerCaseFileName.substring(1);
            criticNetwork.set(size-1, upperCaseFileName);
        }
        return criticNetwork;
    }

    private void failIfConfigurationHasNoCritic(ConfigurationSymbol configurationSymbol) {
        if (!configurationSymbol.getEntryMap().containsKey("critic")) {
            failWithMessage("No critic network file given, but is required for selected algorithm");
        }
    }

    private void failIfActorHasMultipleIO(TrainedArchitecture trainedArchitecture) {
        if (trainedArchitecture.getInputs().size() > 1 || trainedArchitecture.getOutputs().size() > 1) {
            failWithMessage("Actor component with multiple inputs or outputs is not supported by this generator");
        }
    }

    private void failIfArchitectureNotAvailable(ConfigurationSymbol configurationSymbol) {
        if (!configurationSymbol.getTrainedArchitecture().isPresent()) {
            failWithMessage("No architecture symbol found but is required for selected algorithm");
        }
    }

    private void failWithMessage(final String message) {
        Log.error("Critic network generation failed: " + message);
        throw new CriticNetworkGenerationException(message);
    }
}