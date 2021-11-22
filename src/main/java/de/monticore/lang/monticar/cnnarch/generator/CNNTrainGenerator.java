/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

import com.google.common.io.Resources;
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
import java.nio.charset.StandardCharsets;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import static de.monticore.lang.monticar.cnnarch.generator.validation.Constants.ROOT_SCHEMA;
import static de.monticore.lang.monticar.cnnarch.generator.validation.Constants.ROOT_SCHEMA_MODEL_PATH;

public abstract class CNNTrainGenerator {

    private TrainParamSupportChecker trainParamSupportChecker;
    private String generationTargetPath;
    private String instanceName;

    public CNNTrainGenerator(TrainParamSupportChecker trainParamSupportChecker) {
        setGenerationTargetPath("./target/generated-sources-cnnarch/");
        this.trainParamSupportChecker = trainParamSupportChecker;
    }

    public TrainingConfiguration createTrainingConfiguration(Path modelsDirPath, String rootModelName) {
        ConfigurationSymbol configuration = resolveTrainingConfiguration(modelsDirPath, rootModelName);
        setInstanceName(rootModelName);

        URL schemasResource = getClass().getClassLoader().getResource(ROOT_SCHEMA_MODEL_PATH);
        URL url = getClass().getResource(ROOT_SCHEMA_MODEL_PATH + ROOT_SCHEMA);
        System.out.println("urls:");
        System.out.println(url);
        System.out.println(schemasResource);
        List<SchemaDefinitionSymbol> schemaDefinitionSymbols;
        try {
            String content = Resources.toString(url, StandardCharsets.UTF_8);
            assert schemasResource != null;
            FileSystem fileSystem = initFileSystem(schemasResource.toURI());
            System.out.println("fileSystem:");
            System.out.println(fileSystem);
            Path path = fileSystem.getPath(schemasResource.getPath());
            ModelPath modelPath = new ModelPath(path);
            SchemaDefinitionSymbol schema = resolveSchemaDefinition(ROOT_SCHEMA, modelPath);
            SchemaLangCoCoChecker checkerWithAllCoCos = SchemaLangCocoFactory.getCheckerWithAllCoCos();
            checkerWithAllCoCos.checkAll(schema.getSchemaDefinitionNode().get());

            schemaDefinitionSymbols = SchemaLangValidator.resolveSchemaHierarchy(schema, configuration, modelPath);
            List<SchemaViolation> schemaViolations = SchemaLangValidator.validateConfiguration(schemaDefinitionSymbols, configuration);
            if (!schemaViolations.isEmpty()) {
                logViolations(schemaViolations);
                throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
            }
        } catch (SchemaLangException e) {
            throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
        } catch (URISyntaxException e) {
            throw new RuntimeException(String.format("Generation aborted, since there the path '%s' is not valid.",
                schemasResource.toString()));
        } catch (IOException e) {
            throw new RuntimeException(String.format("Generation aborted. File system could not be created."), e);
        }
        return TrainingConfiguration.create(configuration, schemaDefinitionSymbols);
    }

    private FileSystem initFileSystem(URI uri) throws IOException {
        try {
            return FileSystems.getDefault();
        } catch(Exception e) {
            return FileSystems.newFileSystem(uri, Collections.emptyMap());
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
            String message = String.format("Could not resolve schema definition for model '%s'.", rootModelName);
            Log.error(message);
            throw new RuntimeException(message);
        }

        SchemaDefinitionSymbol schemaDefinitionSymbol = compilationUnit.get();
        SchemaLangCoCoChecker checkerWithAllCoCos = SchemaLangCocoFactory.getCheckerWithAllCoCos();
        checkerWithAllCoCos.checkAll(schemaDefinitionSymbol.getSchemaDefinitionNode().get());
        return schemaDefinitionSymbol;
    }

    protected void validateConfiguration(TrainingConfiguration trainingConfiguration, TrainingComponentsContainer trainingComponentsContainer) {

        URL schemasResource = getClass().getClassLoader().getResource("schemas/");
        try {
            assert schemasResource != null;
            Path path = Paths.get(schemasResource.toURI());
            Collection<Violation> violations = SchemaLangValidator.validate(trainingConfiguration.getConfigurationSymbol(), ROOT_SCHEMA,
                    trainingComponentsContainer.getArchitectureComponents(), new ModelPath(path));
            if (!violations.isEmpty()) {
                logViolations(violations);
                throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
            }
        } catch (SchemaLangException e) {
            throw new GenerationAbortedException("Generation aborted due to errors in the training configuration.");
        } catch (URISyntaxException e) {
            throw new RuntimeException(String.format("Generation aborted, since there the path '%s' is not valid.", schemasResource.toString()));
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

    protected void generateFilesFromConfigurationSymbol(TrainingConfiguration trainingConfiguration, TrainingComponentsContainer trainingComponentsContainer) {

        List<FileContent> fileContents = generateStrings(trainingConfiguration, trainingComponentsContainer);
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
    public abstract List<FileContent> generateStrings(TrainingConfiguration trainingConfiguration, TrainingComponentsContainer trainingComponentsContainer);
}