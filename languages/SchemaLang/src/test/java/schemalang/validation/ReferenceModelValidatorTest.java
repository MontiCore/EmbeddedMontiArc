package schemalang.validation;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import conflang._ast.ASTConfiguration;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._symboltable.SchemaDefinitionSymbol;
import schemalang.exception.SchemaLangTechnicalException;
import schemalang.validation.model.ArchitectureComponent;
import schemalang.validation.model.MappingUtils;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.junit.Assert.*;
import static schemalang.ErrorCodes.*;

public class ReferenceModelValidatorTest extends AbstractTest {

    private ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/" +
            "validation/referencemodels"));

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
    }

    @Test
    public void validateReferenceModel() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "ValidateReferenceModel.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/ValidateReferenceModel.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), Maps.newHashMap());

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_01C, String.format(ERROR_MSG_TA_01C, "actor")),
                Violation.create(ERROR_CODE_TA_01C, String.format(ERROR_MSG_TA_01C, "critic"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg1.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void testVAEMatchingPortRange() {

        /* Arrange */
        ASTConfiguration configuration =
                parseConfiguration("src/test/resources/conflang/VAE.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/VAE.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent encoderComponent = parseEMAComponent("src/test/resources/ema/vae/Encoder.ema");
        createEMASymbolTable(encoderComponent);
        ASTComponent decoderComponent = parseEMAComponent("src/test/resources/ema/vae/Decoder.ema");
        createEMASymbolTable(decoderComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("encoder", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) encoderComponent.getSymbol()));
        componentMap.put("decoder", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) decoderComponent.getSymbol()));

        /* Act */
        Collection<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertTrue(violations.isEmpty());
    }

    @Test
    public void connectorsWithIncompatiblePorts() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "ValidateConnections.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/ValidateConnections.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg2/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg2/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_09C, String.format(ERROR_MSG_TA_09C,
                        "actor.output_actor -> critic.input_critic")),
                Violation.create(ERROR_CODE_TA_07C, String.format(ERROR_MSG_TA_07C, "Critic", "Actor",
                        "critic.output_critic -> actor.input_actor")),
                Violation.create(ERROR_CODE_TA_09C, String.format(ERROR_MSG_TA_09C,
                        "critic.output_critic -> actor.input_actor"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg2.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void onePortTypeDiffers() {

        /* Arrange */
        ASTConfiguration configuration =
                parseConfiguration("src/test/resources/conflang/ValidateComponentTypesSingle.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/ValidateComponentTypesSingle.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg3/Actor.ema");
        createEMASymbolTable(actorComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_13C, String.format(ERROR_MSG_TA_13C, "input_actor"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg3.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void inputAndOutputPortTypesAndRangesDiffer() {

        /* Arrange */
        ASTConfiguration configuration =
                parseConfiguration("src/test/resources/conflang/InputAndOutputPortTypesAndRangesDiffer.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/" +
                        "InputAndOutputPortTypesAndRangesDiffer.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg4/Actor.ema");
        createEMASymbolTable(actorComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_13C, String.format(ERROR_MSG_TA_13C, "input_actor")),
                Violation.create(ERROR_CODE_TA_14C, String.format(ERROR_MSG_TA_14C, "input_actor")),
                Violation.create(ERROR_CODE_TA_13C, String.format(ERROR_MSG_TA_13C, "output_actor")),
                Violation.create(ERROR_CODE_TA_14C, String.format(ERROR_MSG_TA_14C, "output_actor"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg4.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void requiredComponentMissing() {

        /* Arrange */
        ASTConfiguration configuration =
                parseConfiguration("src/test/resources/conflang/RequiredComponentMissing.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/RequiredComponentMissing.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg5/Actor.ema");
        createEMASymbolTable(actorComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_01C, String.format(ERROR_MSG_TA_01C, "critic"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg5.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void inputAndOutputPortTypesAndRangesMatch() {

        /* Arrange */
        ASTConfiguration configuration =
                parseConfiguration("src/test/resources/conflang/InputAndOutputPortTypesAndRangesMatch.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/InputAndOutputPortTypesAndRangesMatch.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg6/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg6/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        Collection<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertTrue(violations.isEmpty());
    }

    @Test
    public void portNamesDoNotMatch() {

        /* Arrange */
        ASTConfiguration configuration =
                parseConfiguration("src/test/resources/conflang/PortNamesDoNotMatch.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/PortNamesDoNotMatch.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg7/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg7/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_04C, String.format(ERROR_MSG_TA_04C,
                        "Actor", "input_actor"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg7.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void referenceModelDoesNotExist() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "ReferenceModelDoesNotExist.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/ReferenceModelDoesNotExist.scm");

        Collection<ReferenceModelViolation> violations = null;
        try {
            /* Act */
            createSymbolTable2(parsedModel, modelPath);
            ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();
            violations = ReferenceModelValidator.validate(
                    schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), Maps.newHashMap());
            fail("A SchemaLangTechnicalException was expected, but not thrown.");
        } catch (SchemaLangTechnicalException e) {
            // ignore
            assertEquals("Reference model 'referencemodel.NotExists' in schema definition " +
                    "'ReferenceModelDoesNotExist' could not be resolved.", e.getMessage());
            assertNull(violations);
        }
    }

    @Test
    public void environmentDefinedAsRosInterface() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "EnvironmentDefinedAsRosInterface.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/EnvironmentDefinedAsRosInterface.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg8/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg8/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        Collection<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertTrue(violations.isEmpty());
    }

    @Ignore
    @Test
    public void environmentDefinedAsRosInterfaceNonNested() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "EnvironmentDefinedAsRosInterfaceNonNested.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/EnvironmentDefinedAsRosInterfaceNonNested.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg10/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg10/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_19C, String.format(ERROR_MSG_TA_19C,
                        "environment", "state, terminal, action, reset"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg10.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void multipleReferenceModelsSuccess() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "MultipleReferenceModelsSuccess.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/MultipleReferenceModelsSuccess.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg11/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg11/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        Collection<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertTrue(violations.isEmpty());
    }

    @Test
    public void multipleReferenceModelsFail() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "MultipleReferenceModelsFail.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/MultipleReferenceModelsFail.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg13/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg13/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertEquals(2, violations.size());
        Collection<Violation> expectedErrorsDdpg = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_16C, String.format(ERROR_MSG_TA_16C, "terminal"))
        );
        Collection<Violation> expectedErrorsDdpgWithReward = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_16C, String.format(ERROR_MSG_TA_16C, "terminal, reward"))
        );
        ReferenceModelViolation violationDdpg = new ReferenceModelViolation("ddpg13.DDPG");
        violationDdpg.setViolations(expectedErrorsDdpg);
        assertThat(violations.get(0), equalTo(violationDdpg));

        ReferenceModelViolation violationDdpgWithReward = new ReferenceModelViolation("ddpg13.DDPGWithReward");
        violationDdpgWithReward.setViolations(expectedErrorsDdpgWithReward);
        assertThat(violations.get(1), equalTo(violationDdpgWithReward));
    }

    @Test
    public void inputAndOutputPortTypesAndRangesDontMatch() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "InputAndOutputPortTypesAndRangesDontMatch.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel = parse("src/test/resources/schemalang/validation/" +
                "referencemodels/InputAndOutputPortTypesAndRangesDontMatch.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg14/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg14/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_07C, String.format(ERROR_MSG_TA_07C, "Actor", "Critic",
                        "actor.output_actor -> critic.input_critic")),
                Violation.create(ERROR_CODE_TA_11C, String.format(ERROR_MSG_TA_11C, "Actor", "Critic",
                        "actor.output_actor -> critic.input_critic")),
                Violation.create(ERROR_CODE_TA_11C, String.format(ERROR_MSG_TA_11C, "Critic", "Actor",
                        "critic.output_critic -> actor.input_actor"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg14.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void portsOfPrimitiveTypeAreChecked() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "PortsOfPrimitiveTypeAreChecked.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel = parse("src/test/resources/schemalang/validation/" +
                "referencemodels/PortsOfPrimitiveTypeAreChecked.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg15/Actor.ema");
        createEMASymbolTable(actorComponent);
        ASTComponent criticComponent = parseEMAComponent("src/test/resources/ema/ddpg15/Critic.ema");
        createEMASymbolTable(criticComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));
        componentMap.put("critic", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) criticComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertEquals(1, violations.size());

        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_20C, String.format(ERROR_MSG_TA_20C,
                        "primitive_input_port", "Actor", "B", "Q")),
                Violation.create(ERROR_CODE_TA_20C, String.format(ERROR_MSG_TA_20C,
                        "primitive_output_port", "Actor", "Q", "B")),
                Violation.create(ERROR_CODE_TA_20C, String.format(ERROR_MSG_TA_20C,
                        "primitive_input_port", "Critic", "Q", "B")),
                Violation.create(ERROR_CODE_TA_20C, String.format(ERROR_MSG_TA_20C,
                        "primitive_output_port", "Critic", "B", "Q"))
        );

        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg15.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void portOfVectorTypeIsChecked() {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/" +
                "PortOfVectorTypeIsChecked.conf");
        createEMASymbolTable(configuration);

        ASTSchemaLangCompilationUnit parsedModel = parse("src/test/resources/schemalang/validation/" +
                "referencemodels/PortOfVectorTypeIsChecked.scm");
        createSymbolTable2(parsedModel, modelPath);
        ASTSchemaDefinition schemaDefinition = parsedModel.getSchemaDefinition();

        ASTComponent actorComponent = parseEMAComponent("src/test/resources/ema/ddpg16/Actor.ema");
        createEMASymbolTable(actorComponent);

        HashMap<String, ArchitectureComponent> componentMap = Maps.newHashMap();
        componentMap.put("actor", MappingUtils.createArchitectureComponent(
                (EMAComponentSymbol) actorComponent.getSymbol()));

        /* Act */
        List<ReferenceModelViolation> violations = ReferenceModelValidator.validate(
                schemaDefinition.getSchemaDefinitionSymbol(), configuration.getConfigurationSymbol(), componentMap);

        /* Assert */
        assertNotNull(violations);
        assertEquals(1, violations.size());
        Collection<Violation> expectedErrors = Lists.newArrayList(
                Violation.create(ERROR_CODE_TA_21C, String.format(ERROR_MSG_TA_21C,
                        "output_actor", "Actor", "1"))
        );
        ReferenceModelViolation violation = new ReferenceModelViolation("ddpg16.DDPG");
        violation.setViolations(expectedErrors);
        assertThat(violations.get(0), equalTo(violation));
    }

    @Test
    public void nestedConfigurationEntryInReferenceModel() {
        /* Arrange */
        final ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/DefinedNestedEntriesInReferenceModel.conf");
        createEMASymbolTable(configuration);
        final ASTSchemaLangCompilationUnit parsedModel =
                parse("src/test/resources/schemalang/validation/referencemodels/SchemaWithReferenceModel.scm");
        createSymbolTable2(parsedModel, modelPath);
        assertNotNull(configuration);
        /* Act */
        final SchemaDefinitionSymbol schemaDefinitionSymbol = parsedModel.getSchemaDefinition().getSchemaDefinitionSymbol();
        final List<SchemaViolation> schemaViolations = SchemaDefinitionValidator.validateConfiguration(Lists.newArrayList(schemaDefinitionSymbol), configuration.getConfigurationSymbol());
        /* Assert */
        assertTrue(schemaViolations.isEmpty());
    }

    /***
     * three violations are expected
     * one for the undefined nested entry in the reference model
     * one for the undefined simple entry  in the reference model
     * one for the defined nested entry which has an undefined nested entry
     */
    @Test
    public void nestedConfigurationEntryNotInReferenceModel() {
        /* Arrange */
        final ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/UndefinedNestedEntriesInReferenceModel.conf");
        createEMASymbolTable(configuration);
        final ASTSchemaLangCompilationUnit parsedModel = parse("src/test/resources/schemalang/validation/referencemodels/SchemaWithReferenceModel.scm");
        createSymbolTable2(parsedModel, modelPath);
        assertNotNull(configuration);
        /* Act */
        final SchemaDefinitionSymbol schemaDefinitionSymbol = parsedModel.getSchemaDefinition().getSchemaDefinitionSymbol();
        final List<SchemaViolation> schemaViolations = SchemaDefinitionValidator.validateConfiguration(Lists.newArrayList(schemaDefinitionSymbol), configuration.getConfigurationSymbol());
        /* Assert */
        assertEquals(3, schemaViolations.size());
    }
}