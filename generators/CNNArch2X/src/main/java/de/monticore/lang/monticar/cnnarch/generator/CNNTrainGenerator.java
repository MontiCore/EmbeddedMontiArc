/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import conflang._cocos.ConfLangCoCoChecker;
import conflang._cocos.ConfLangCocoFactory;
import conflang._symboltable.ConfLangLanguage;
import conflang._symboltable.ConfigurationSymbol;
import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.monticore.lang.monticar.generator.FileContent;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import schemalang._cocos.SchemaLangCoCoChecker;
import schemalang._cocos.SchemaLangCocoFactory;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang._symboltable.SchemaLangLanguage;
import schemalang.validation.ReferenceModelViolation;
import schemalang.validation.SchemaLangValidator;
import schemalang.validation.SchemaViolation;
import schemalang.validation.Violation;
import schemalang.validation.exception.SchemaLangException;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.util.*;

import static de.monticore.lang.monticar.cnnarch.generator.validation.Constants.ROOT_SCHEMA_MODEL_PATH;

public abstract class CNNTrainGenerator {

    private TrainParamSupportChecker trainParamSupportChecker;
    private String generationTargetPath;
    private String instanceName;

    public CNNTrainGenerator(TrainParamSupportChecker trainParamSupportChecker) {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
        this.trainParamSupportChecker = trainParamSupportChecker;
    }

    public TrainingConfiguration createTrainingConfiguration(Path modelsDirPath, String rootModelName,
                                                             Path outputPath) {
        ConfigurationSymbol configuration = resolveTrainingConfiguration(modelsDirPath, rootModelName);
        setInstanceName(rootModelName);

        URL schemasResource = getClass().getClassLoader().getResource(ROOT_SCHEMA_MODEL_PATH);
        List<SchemaDefinitionSymbol> schemaDefinitionSymbols;
        try {
            assert schemasResource != null;
            FileSystem fileSystem = initFileSystem(schemasResource.toURI());
            Path path = fileSystem.getPath(schemasResource.getPath());
            if (outputPath != null) {
                path = outputPath.resolve(ROOT_SCHEMA_MODEL_PATH);
            }
            ModelPath modelPath = new ModelPath(path);
            System.out.println(String.format("Model path for schema resolution: %s", modelPath.toString()));

            SchemaDefinitionSymbol schema = resolveSchemaDefinition("General", modelPath);
            SchemaLangCoCoChecker checkerWithAllCoCos = SchemaLangCocoFactory.getCheckerWithAllCoCos();
            checkerWithAllCoCos.checkAll(schema.getSchemaDefinitionNode().get());
            schemaDefinitionSymbols = SchemaLangValidator.resolveSchemaHierarchy(schema, configuration, modelPath);
            List<SchemaViolation> schemaViolations = SchemaLangValidator.validateConfiguration(schemaDefinitionSymbols, configuration);
            if (!schemaViolations.isEmpty()) {
                logViolations(schemaViolations);
                throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
            }
        } catch (SchemaLangException | URISyntaxException | IOException e) {
            Log.error(e.getMessage(), e);
            throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
        }
        return TrainingConfiguration.create(configuration, schemaDefinitionSymbols);
    }

    private FileSystem initFileSystem(URI uri) throws IOException {
        try {
            FileSystem aDefault = FileSystems.getDefault();
            return aDefault;
        } catch(Exception e) {
            Map<String, String> env = new HashMap<>();
            env.put("create", "true");
            FileSystem fileSystem = FileSystems.newFileSystem(uri, env);
            return fileSystem;
        }
    }

    private ConfigurationSymbol resolveTrainingConfiguration(Path modelsDirPath, String rootModelName) {
        final ModelPath mp = new ModelPath(modelsDirPath);
        GlobalScope scope = new GlobalScope(mp, new ConfLangLanguage());
        Optional<ConfigurationSymbol> configurationSymbolOpt = scope.resolve(rootModelName, ConfigurationSymbol.KIND);
        if (!configurationSymbolOpt.isPresent()) {
            String message = String.format("Could not resolve training configuration for model '%s'.", rootModelName);
            Log.error(message);
            throw new RuntimeException(message);
        }
        ConfigurationSymbol configurationSymbol = configurationSymbolOpt.get();

        setInstanceName(configurationSymbol.getFullName());
        ConfLangCoCoChecker checkerWithAllCoCos = ConfLangCocoFactory.createCheckerWithAllCoCos();
        checkerWithAllCoCos.checkAll(configurationSymbol.getConfigurationNode().get());
        trainParamSupportChecker.check(configurationSymbolOpt.get());

        return configurationSymbolOpt.get();
    }

    private SchemaDefinitionSymbol resolveSchemaDefinition(String rootModelName, ModelPath modelPath) {

        ModelingLanguageFamily family = new ModelingLanguageFamily();
        family.addModelingLanguage(new SchemaLangLanguage());
        family.addModelingLanguage(new EmbeddedMontiArcLanguage());
        GlobalScope globalScope = new GlobalScope(modelPath, family);
        Optional<SchemaDefinitionSymbol> compilationUnit = globalScope.resolve(rootModelName,
                SchemaDefinitionSymbol.KIND);

        if (!compilationUnit.isPresent()) {
            String message = String.format("Could not resolve schema definition for model '%s' in model path '%s'.", rootModelName, modelPath);
            Log.error(message);
            throw new RuntimeException(message);
        }

        SchemaDefinitionSymbol schemaDefinitionSymbol = compilationUnit.get();
        SchemaLangCoCoChecker checkerWithAllCoCos = SchemaLangCocoFactory.getCheckerWithAllCoCos();
        checkerWithAllCoCos.checkAll(schemaDefinitionSymbol.getSchemaDefinitionNode().get());
        return schemaDefinitionSymbol;
    }

    protected void validateConfiguration(TrainingConfiguration trainingConfiguration,
                                         TrainingComponentsContainer trainingComponentsContainer,
                                         Path outputPath) {

        URL schemasResource = getClass().getClassLoader().getResource(ROOT_SCHEMA_MODEL_PATH);
        try {
            assert schemasResource != null;
            FileSystem fileSystem = initFileSystem(schemasResource.toURI());
            Path path = fileSystem.getPath(schemasResource.getPath());
            if (outputPath != null) {
                path = outputPath.resolve(ROOT_SCHEMA_MODEL_PATH);
            }
            Collection<Violation> violations = SchemaLangValidator.validate(
                    trainingConfiguration.getConfigurationSymbol(), "General",
                    trainingComponentsContainer.getArchitectureComponents(), new ModelPath(path));
            if (!violations.isEmpty()) {
                logViolations(violations);
                throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
            }
        } catch (SchemaLangException e) {
            Log.error(e.getMessage(), e);
            throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
        } catch (URISyntaxException e) {
            throw new RuntimeException(String.format("Generation aborted, since there the path '%s' is not valid.", schemasResource.toString()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private void logViolations(Collection<? extends Violation> violations) {
        for (Violation violation : violations) {
            if (violation instanceof ReferenceModelViolation) {
                ReferenceModelViolation referenceModelViolation = (ReferenceModelViolation) violation;
                Log.error(referenceModelViolation.toString());
                Collection<Violation> rmViolations = referenceModelViolation.getViolations();
                logViolations(rmViolations);
                continue;
            }
            Log.error(violation.toString());
        }
    }

    protected void generateFilesFromConfigurationSymbol(TrainingConfiguration trainingConfiguration,
                                                        TrainingComponentsContainer trainingComponentsContainer,
                                                        Path outputPath) {

        List<FileContent> fileContents = generateStrings(trainingConfiguration, trainingComponentsContainer, outputPath);
        GeneratorCPP genCPP = new GeneratorCPP();
        genCPP.setGenerationTargetPath(getGenerationTargetPath());

        try {
            for (FileContent fileContent : fileContents) {
                genCPP.generateFile(fileContent);
            }
        } catch (IOException var8) {
            Log.error("CNNTrainer file could not be generated" + var8.getMessage());
        }
    }

    public String getInstanceName() {
        String parsedInstanceName = this.instanceName.replace('.', '_').replace('[', '_').replace(']', '_');
        parsedInstanceName = parsedInstanceName.substring(0, 1).toLowerCase() + parsedInstanceName.substring(1);
        return parsedInstanceName;
    }

    public void setInstanceName(String instanceName) {
        this.instanceName = instanceName;
    }

    public String getGenerationTargetPath() {
        if (generationTargetPath.charAt(generationTargetPath.length() - 1) != '/') {
            this.generationTargetPath = generationTargetPath + "/";
        }
        return generationTargetPath;
    }

    public void setGenerationTargetPath(String generationTargetPath) {
        this.generationTargetPath = generationTargetPath;
    }

    private static void quitGeneration() {
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    public abstract void generate(Path modelsDirPath, String rootModelNames);

    //check cocos with CNNTrainCocos.checkAll(configuration) before calling this method.
    public abstract List<FileContent> generateStrings(TrainingConfiguration trainingConfiguration,
                                                      TrainingComponentsContainer trainingComponentsContainer,
                                                      Path outputPath);
}