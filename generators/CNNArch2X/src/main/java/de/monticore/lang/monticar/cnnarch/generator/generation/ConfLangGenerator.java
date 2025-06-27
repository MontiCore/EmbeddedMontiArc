package de.monticore.lang.monticar.cnnarch.generator.generation;

import conflang._ast.ASTConfLangCompilationUnit;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.lang.monticar.cnnarch.generator.CNNTrainGenerator;
import de.monticore.lang.monticar.cnnarch.generator.TrainParamSupportChecker;
import de.monticore.lang.monticar.cnnarch.generator.configuration.ConfLangGeneratorConfiguration;
import de.monticore.lang.monticar.cnnarch.generator.configuration.Template;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingComponentsContainer;
import de.monticore.lang.monticar.cnnarch.generator.training.TrainingConfiguration;
import de.monticore.lang.monticar.generator.FileContent;
import org.apache.commons.lang3.NotImplementedException;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

/***
 * Generator to generate code representation of ConfLang configuration
 */
public class ConfLangGenerator extends CNNTrainGenerator {

    private final GeneratorEngine generatorEngine;

    public ConfLangGenerator() {
        this(null);
    }

    protected ConfLangGenerator(final TrainParamSupportChecker trainParamSupportChecker) {
        super(trainParamSupportChecker);
        GeneratorSetup generatorSetup = new GeneratorSetup();
        generatorSetup.setTracing(false);
        generatorSetup.setGlex(ConfLangGeneratorConfiguration.getGlobalExtensionManagement());
        generatorSetup.setOutputDirectory(new File("target/generated-sources/configurations"));
        generatorEngine = new GeneratorEngine(generatorSetup);
    }

    public void generatePythonTrainingConfiguration(final ASTConfLangCompilationUnit configurationModel, String generatedConfigurationName) {
        final Path fileName = Paths.get(generatedConfigurationName + ".py");
        generatorEngine.generate(Template.TRAINING_CONFIGURATION_CLASS.getTemplateName(), fileName, configurationModel, configurationModel);

    }

    @Deprecated
    @Override
    public void generate(final Path modelsDirPath, final String rootModelNames) {

    }

    @Deprecated
    @Override
    public List<FileContent> generateStrings(final TrainingConfiguration trainingConfiguration, final TrainingComponentsContainer trainingComponentsContainer, final Path outputPath) {
       throw new NotImplementedException("This is method is deprecated.");
    }

}
