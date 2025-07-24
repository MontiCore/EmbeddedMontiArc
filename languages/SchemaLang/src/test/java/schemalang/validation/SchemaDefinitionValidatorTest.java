package schemalang.validation;

import com.google.common.collect.Lists;
import conflang._ast.ASTConfiguration;
import conflang._cocos.ConfLangCoCoChecker;
import conflang._cocos.ConfLangCocoFactory;
import conflang._symboltable.ConfLangLanguage;
import conflang._symboltable.ConfLangSymbolTableCreator;
import conflang._symboltable.ConfigurationScope;
import de.monticore.io.paths.ModelPath;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;
import org.hamcrest.CoreMatchers;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang.exception.SchemaLangTechnicalException;
import schemalang.validation.exception.SchemaLangException;

import java.nio.file.Paths;
import java.util.Collection;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.junit.Assert.*;
import static schemalang.ErrorCodes.*;

public class SchemaDefinitionValidatorTest extends AbstractTest {

    private ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/validation"));
    private ConfLangCoCoChecker checker = ConfLangCocoFactory.createCheckerWithAllCoCos();


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
    public void validateConfigurationBySchemaLinksDefinition() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/SupervisedLearning.conf");
        createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(), "General", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void configurationEntriesWithIncompatibleValues() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/ConfigurationEntryWithIncompatibleValue.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "ConfigurationEntryWithIncompatibleValue", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(21, schemaViolations.size());
        Collection<SchemaViolation> expectedErrors = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "undefined"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_12C, String.format(ERROR_MSG_SL_12C, "undefinedEnumConstant", "my_enum", "value1, value2"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "boolean_property", "B"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "boolean_list", "B*"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "natural_numbers_property", "N1"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "natural_numbers_list", "N1*"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "natural_numbers_with_range", "(10:20)"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_15C, String.format(ERROR_MSG_SL_15C, "natural_numbers_with_range_and_scale", "(5)"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "integer_property", "Z"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "integer_list", "Z*"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "integer_with_range", "(-10:10)"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_15C, String.format(ERROR_MSG_SL_15C, "integer_with_range_and_scale", "(2)"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "rational_property", "Q"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "rational_list", "Q*"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "rational_with_range", "(-1:1)"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_15C, String.format(ERROR_MSG_SL_15C, "rational_with_range_and_scale", "(0.1)"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "string_property", "string"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "string_list", "string*"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_11C, String.format(ERROR_MSG_SL_11C, "component_property", "component"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "undefined_custom_property"), "ConfigurationEntryWithIncompatibleValue"),
                SchemaViolation.create(ERROR_CODE_SL_12C, String.format(ERROR_MSG_SL_12C, "undefined", "custom_property", "defined_instance"), "ConfigurationEntryWithIncompatibleValue")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedErrors));
    }

    @Test
    public void integerProperties() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/IntegerProperties.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "IntegerProperties", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void decimalProperties() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/DecimalProperties.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "DecimalProperties", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void stringProperties() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/StringProperties.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "StringProperties", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void validateConfigurationBySeveralSchemaLinksDefinitions() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/DDPG.conf");
        createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(), "SingleSchemaLinksDefinition", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void validateBySchemaLinks() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/DDPGSchemaLinks.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "General", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void validateConfigurationBySchemaLinksDefinitionWithoutLinkingParameters() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/WithoutLinkingParameters.conf");
        createConfLangSymbolTable(configuration);

        /* Act */
        SchemaLangValidator.validate(configuration.getConfigurationSymbol(), "WithoutLinkingParameters", null, modelPath);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void optimizerImported() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/Optimizers.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(), "ImportedOptimizer", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void importSchemaNotAvailable() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/Optimizers.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act and assert */
        try {
            SchemaLangValidator.validate(configuration.getConfigurationSymbol(), "ImportOptimizerNotAvailable", null, modelPath);
            fail("A SchemaLangTechnicalException was expected, but not thrown.");
        } catch (SchemaLangTechnicalException e) {
            // ignore
            assertEquals(0, Log.getErrorCount());
        }
    }

    @Test
    public void optimizerNotImported() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/NotImportedOptimizer.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "NotImportedOptimizer", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(2, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_13C, String.format(ERROR_MSG_SL_13C, "optimizer_type", "critic_optimizer"), "NotImportedOptimizer"),
                SchemaViolation.create(ERROR_CODE_SL_13C, String.format(ERROR_MSG_SL_13C, "optimizer_type", "actor_optimizer"), "NotImportedOptimizer")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    @Test
    public void requiredEntries() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/RequiredEntries.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "RequiredEntries", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(3, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_16C, String.format(ERROR_MSG_SL_16C, "required"), "RequiredEntries"),
                SchemaViolation.create(ERROR_CODE_SL_16C, String.format(ERROR_MSG_SL_16C, "required_nested"), "RequiredEntries"),
                SchemaViolation.create(ERROR_CODE_SL_16C, String.format(ERROR_MSG_SL_16C, "required_enum"), "RequiredEntries")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    @Test
    public void extendedComplexProperty() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/ExtendedComplexProperty.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "ExtendedComplexProperty", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void overriddenObjectDefinition() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/OverriddenComplexProperty.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "OverriddenComplexProperty", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(4, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "epsilon"), "OverriddenComplexProperty"),
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "epsilon"), "OverriddenComplexProperty"),
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "epsilon"), "OverriddenComplexProperty"),
                SchemaViolation.create(ERROR_CODE_SL_12C, String.format(ERROR_MSG_SL_12C, "epsgreedy", "strategy_epsgreedy", "ornstein_uhlenbeck, gaussian"), "OverriddenComplexProperty")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    @Test
    public void extendedComplexPropertyInvalidNestedProperties() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/ExtendedComplexPropertyInvalidNestedProperties.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "ExtendedComplexProperty", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(2, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "mu"), "ExtendedComplexProperty"),
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "noise_variance"), "ExtendedComplexProperty")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    @Test
    public void undefinedNestedEntries() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/UndefinedNestedEntries.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "UndefinedNestedEntries", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(1, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_10C, String.format(ERROR_MSG_SL_10C, "is_not_defined"), "UndefinedNestedEntries")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    @Test
    public void requiresRulesNotSatisfied() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/RequiresRulesNotSatisfied.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "RequiresRulesNotSatisfied", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(2, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_21C, String.format(ERROR_MSG_SL_21C, "batch_size", "num_epoch"), "RequiresRulesNotSatisfied"),
                SchemaViolation.create(ERROR_CODE_SL_21C, String.format(ERROR_MSG_SL_21C, "normalize", "num_epoch"), "RequiresRulesNotSatisfied")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    @Test
    public void requiresRulesSatisfied() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/RequiresRulesSatisfied.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "RequiresRulesSatisfied", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void ranges() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/Ranges.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "Ranges", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(0, schemaViolations.size());
    }

    @Test
    public void notInRanges() throws SchemaLangException {

        /* Arrange */
        ASTConfiguration configuration = parseConfiguration("src/test/resources/conflang/NotInRanges.conf");
        ConfigurationScope confLangSymbolTable = createConfLangSymbolTable(configuration);

        /* Act */
        Collection<Violation> schemaViolations = SchemaLangValidator.validate(configuration.getConfigurationSymbol(),
                "NotInRanges", null, modelPath);

        /* Assert */
        assertNotNull(configuration);
        assertNotNull(confLangSymbolTable);
        assertEquals(8, schemaViolations.size());
        Collection<SchemaViolation> expectedSchemaViolations = Lists.newArrayList(
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "closed_interval_min", "[-2:2]"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "closed_interval_max", "[-2:2]"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "open_interval_min", "(-2:2)"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "open_interval_max", "(-2:2)"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "rightopen_interval_min", "[-2:2)"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "rightopen_interval_max", "[-2:2)"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "leftopen_interval_min", "(-2:2]"), "NotInRanges"),
                SchemaViolation.create(ERROR_CODE_SL_14C, String.format(ERROR_MSG_SL_14C, "leftopen_interval_max", "(-2:2]"), "NotInRanges")
        );
        assertThat(schemaViolations, CoreMatchers.is(expectedSchemaViolations));
    }

    private static ConfigurationScope createConfLangSymbolTable(ASTConfiguration confLangConfiguration) {
        final ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/cocos"));
        final ConfLangLanguage confLangLanguage = new ConfLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(confLangLanguage.getResolvingFilters());

        ConfLangSymbolTableCreator symbolTableCreator = new ConfLangSymbolTableCreator(resolverConfiguration, new GlobalScope(modelPath, confLangLanguage));
        return (ConfigurationScope) symbolTableCreator.createFromAST(confLangConfiguration);
    }
}