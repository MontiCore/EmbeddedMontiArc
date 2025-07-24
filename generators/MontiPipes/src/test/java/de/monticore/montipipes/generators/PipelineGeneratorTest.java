package de.monticore.montipipes.generators;

import com.google.common.collect.Lists;
import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfigurationEntry;
import conflang._parser.ConfLangParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.ExecutionSemantics;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.montipipes.config.ExecutionScriptConfiguration;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

class PipelineGeneratorTest {

    @Test
    void oneTimelinearPipelineExecution() throws IOException {
        //prepare data
        ConfLangParser confLangParser = new ConfLangParser();
        ASTConfLangCompilationUnit pipelineConfiguration = confLangParser.parse("src/test/resources/models/NetworkPipeline.conf").get();
        final TaggingResolver symTab = SymtabCreator.createSymTabForLanguage(new EmbeddedMontiArcLanguage(), "src/test/resources");
        final EMAComponentInstanceSymbol pipelineExecutionSemantic =
                symTab.<EMAComponentInstanceSymbol>resolve("models.pipeline", EMAComponentInstanceSymbol.KIND).orElseThrow(IllegalStateException::new);
        new ExecutionSemantics(symTab, pipelineExecutionSemantic).addExecutionSemantics();
        final List<ASTConfigurationEntry> pipelineConfigurationEntryList = pipelineConfiguration.getConfiguration().getConfigurationEntryList();
        final List<String> importStatements = Lists.newArrayList("Supervised_Schema_API", "Training_Configuration_LeNetNet", "CNNCreator_LeNet");
        final List<String> cliArguments = Lists.newArrayList("tracking_backends", "mlflow_run_id", "mlflow_tracking_uri");
        final PipelineGenerator pipelineGenerator = new PipelineGenerator();
        pipelineGenerator.setSchemaAPIName("Supervised_Schema_API");
        pipelineGenerator.setTrainingConfigurationName("Training_Configuration_LeNetNet");
        pipelineGenerator.addScriptConfiguration(ExecutionScriptConfiguration.MODEL_DIRECTORY, "./model/mnist/");
        pipelineGenerator.generatePipelineExecutor(pipelineConfigurationEntryList, pipelineExecutionSemantic, importStatements, cliArguments);
        final String expected = String.join("\n", Files.readAllLines(Paths.get("src/test/resources/pipeline_scripts/Pipeline_Executor.py")));
        final String actual = String.join("\n", Files.readAllLines(Paths.get(pipelineGenerator.outputPath.toString(), "Pipeline_Executor.py")));
        assertEquals(expected, actual);
    }
}