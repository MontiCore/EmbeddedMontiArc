package conflang._symboltable;

import com.google.common.collect.Lists;
import conflang.AbstractTest;
import conflang._ast.ASTConfLangCompilationUnit;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class BasicConfigurationEntrySymbolTest extends AbstractTest {

    private static final String MODEL_PATH = "src/test/resources/conflang/_symboltable";
    private static ASTConfLangCompilationUnit compilationUnit;

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);

        compilationUnit = parse("src/test/resources/conflang/_symboltable/SimpleConfigurationEntrySymbol.conf");
    }

    @Before
    public void before() {
        Log.clearFindings();
    }

    @Test
    public void getValueForChar() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("char_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals('x', configurationEntry.getValue());
    }

    @Test
    public void getValueForComponent() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("component_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals("de.monticar.conflang.ConfLangLanguage", configurationEntry.getValue());
    }

    @Test
    public void getValueForTypeless() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("typeless_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals("typeless", configurationEntry.getValue());
    }

    @Test
    public void getValueForString() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("string_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals("Hello World", configurationEntry.getValue());
    }

    @Test
    public void getValueForNull() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("null_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(null, configurationEntry.getValue());
    }

    @Test
    public void getValueForBoolean() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("boolean_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(true, configurationEntry.getValue());
    }

    @Test
    public void getValueForDouble() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("double_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Double(0.25), configurationEntry.getValue());
    }

    @Test
    public void getValueForFloat() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("float_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Float(0.25f), configurationEntry.getValue());
    }

    @Test
    public void getValueForInteger() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("integer_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Integer(100), configurationEntry.getValue());
    }

    @Test
    public void getValueForLong() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("long_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Long(100000L), configurationEntry.getValue());
    }

    @Test
    public void getValueForList() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("list_entry", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(Lists.newArrayList(1, 2, 3, 4), configurationEntry.getValue());
    }

    @Test
    public void isWithinScopedOfNestedConfiguration() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> nestedConfigurationEntryOpt = symbolTable.resolve("nested_entry", NestedConfigurationEntrySymbol.KIND);
        NestedConfigurationEntrySymbol nestedConfigurationEntry = nestedConfigurationEntryOpt.get();
        Optional<ConfigurationEntrySymbol> configurationEntryOpt = nestedConfigurationEntry.getConfigurationEntryOfKind("entry_within_nested", ConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        ConfigurationEntrySymbol configurationEntry= configurationEntryOpt.get();
        assertEquals("entry_within_nested", configurationEntry.getName());
        assertTrue(configurationEntry.isChildOfNestedConfiguration());
    }
}