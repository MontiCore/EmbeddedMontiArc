package de.monticore.montipipes.generators;

import com.google.common.collect.Lists;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTConfigurationEntryTOP;
import conflangliterals._ast.ASTTypelessLiteral;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.montipipes.config.ExecutionScriptConfiguration;
import de.monticore.montipipes.config.Template;
import de.monticore.montipipes.util.TemplateUtil;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

public class PipelineGenerator {

    public Path getOutputPath() {
        return outputPath;
    }

    protected final Path outputPath = Paths.get("target/generated-sources/");
    private final GeneratorSetup generatorSetup = new GeneratorSetup();
    private final GeneratorEngine generatorEngine = new GeneratorEngine(generatorSetup);

    private String schemaAPIName;

    private String trainingConfigurationName;

    private Map<ExecutionScriptConfiguration, String> executionScriptConfigurations = new HashMap<>();

    public void setSchemaAPIName(final String schemaAPIName) {
        this.schemaAPIName = schemaAPIName;
    }

    public void setTrainingConfigurationName(final String trainingConfigurationName) {
        this.trainingConfigurationName = trainingConfigurationName;
    }

    public PipelineGenerator() {
        generatorSetup.setTracing(false);
        generatorSetup.setOutputDirectory(outputPath.toFile());
        generatorSetup.setGlex(new GlobalExtensionManagement());
    }

    public void addScriptConfigurations(Map<ExecutionScriptConfiguration, String> executionScriptConfigurations) {
        this.executionScriptConfigurations.putAll(executionScriptConfigurations);
    }

    public void addScriptConfiguration(ExecutionScriptConfiguration executionScriptConfiguration, final String configurationValue) {
        this.executionScriptConfigurations.put(executionScriptConfiguration, configurationValue);
    }

    public String getScriptConfigurationValue(ExecutionScriptConfiguration executionScriptConfiguration) {
        return this.executionScriptConfigurations.get(executionScriptConfiguration);
    }

    /***
     *
     * @param pipelineConfigurationEntries
     * @param semanticAugmentedPipelineModel
     * @param importedDependencies : generated artefacts to be imported: schema api, network, training configuration
     */
    public void generatePipelineExecutor(final List<ASTConfigurationEntry> pipelineConfigurationEntries,
            final EMAComponentInstanceSymbol semanticAugmentedPipelineModel,
            final List<String> importedDependencies,
            final List<String> cliArguments) {
        final ASTComponent pipelineComponent = (ASTComponent) semanticAugmentedPipelineModel.getComponentType().getReferencedSymbol().getAstNode().orElseThrow(IllegalStateException::new);
        final String importsBlock = createImportStatements(pipelineConfigurationEntries, importedDependencies);
        final LinkedList<EMAComponentInstanceSymbol> pipelineSteps = Find.allAtomicOrNVComponents(semanticAugmentedPipelineModel);
        sortPipelineSteps(pipelineSteps);
        generatorEngine.generate(Template.PIPELINE_EXECUTION.getTemplateName(), Paths.get("Pipeline_Executor.py"), pipelineComponent, importsBlock,
                pipelineConfigurationEntries, pipelineSteps, pipelineComponent, TemplateUtil.getModelDirNameAndValue(executionScriptConfigurations).getValue(),
                schemaAPIName, trainingConfigurationName, cliArguments);
    }

    private static String createImportStatements(final List<ASTConfigurationEntry> pipelineImplementations, final List<String> importedDependencies) {
        List<String> importStatements = Lists.newArrayList();
        importedDependencies.forEach(dependency -> importStatements.add(createPythonImportStatement(dependency)));
        pipelineImplementations.stream().map(ASTConfigurationEntryTOP::getValue)
                .filter(value -> value instanceof ASTTypelessLiteral).map(astSignedLiteral -> (ASTTypelessLiteral) astSignedLiteral)
                .forEach(astTypelessLiteral -> importStatements.add(createPythonImportStatement(astTypelessLiteral.getValue())));
        return String.join("\n", importStatements);
    }

    private static String createPythonImportStatement(final String dependency) {
        return String.format("from %s import %s", dependency, dependency);
    }

    private void sortPipelineSteps(LinkedList<EMAComponentInstanceSymbol> unorderedPipelineSteps) {
        final Comparator<EMAComponentInstanceSymbol> comparator = (EMAComponentInstanceSymbol comp1, EMAComponentInstanceSymbol comp2) -> Integer.compareUnsigned(comp1.getOrderOutput().get(0), comp2.getOrderOutput().get(0));
        unorderedPipelineSteps.sort(comparator);
    }
}
