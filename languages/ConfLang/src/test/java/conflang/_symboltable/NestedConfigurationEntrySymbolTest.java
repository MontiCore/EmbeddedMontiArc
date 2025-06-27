package conflang._symboltable;

import com.google.common.collect.Lists;
import conflang.AbstractTest;
import conflang._ast.ASTConfLangCompilationUnit;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.List;
import java.util.Optional;

import static org.junit.Assert.*;

public class NestedConfigurationEntrySymbolTest extends AbstractTest {

    private static final String MODEL_PATH = "src/test/resources/conflang/_symboltable";
    private static ASTConfLangCompilationUnit compilationUnit;

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);

        compilationUnit = parse("src/test/resources/conflang/_symboltable/NestedConfigurationEntrySymbol.conf");
    }

    @Before
    public void before() {
        Log.clearFindings();
    }

    @Test
    public void getValueForChar() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("char_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals('x', configurationEntry.getValue());
    }

    @Test
    public void getValueForComponent() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("component_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals("de.monticar.conflang.ConfLangLanguage", configurationEntry.getValue());
    }

    @Test
    public void getValueForTypeless() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("typeless_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals("typeless", configurationEntry.getValue());
    }

    @Test
    public void getValueForString() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("string_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals("Hello World", configurationEntry.getValue());
    }

    @Test
    public void getValueForNull() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("null_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(null, configurationEntry.getValue());
    }

    @Test
    public void getValueForBoolean() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("boolean_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(true, configurationEntry.getValue());
    }

    @Test
    public void getValueForDouble() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("double_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Double(0.25), configurationEntry.getValue());
    }

    @Test
    public void getValueForFloat() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("float_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Float(0.25f), configurationEntry.getValue());
    }

    @Test
    public void getValueForInteger() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("integer_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Integer(100), configurationEntry.getValue());
    }

    @Test
    public void getValueForLong() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("long_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(new Long(100000L), configurationEntry.getValue());
    }

    @Test
    public void getValueForList() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("list_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();
        assertEquals(Lists.newArrayList(1, 2, 3, 4), configurationEntry.getValue());
    }

    @Test
    public void getAllConfigurationEntries() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("nested_with_entries", NestedConfigurationEntrySymbol.KIND);
        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();

        List<ConfigurationEntry> allConfigurationEntries = configurationEntry.getAllConfigurationEntries();

        /* Assert. */
        assertNotNull(allConfigurationEntries);
        assertEquals(3, allConfigurationEntries.size());
        assertTrue(allConfigurationEntries.stream().anyMatch(e -> "entry1".equals(e.getName())));
        assertTrue(allConfigurationEntries.stream().anyMatch(e -> "entry2".equals(e.getName())));
        assertTrue(allConfigurationEntries.stream().anyMatch(e -> "entry3".equals(e.getName())));
    }

    @Test
    public void getAllConfigurationEntriesWithNoConfigurationEntries() {

        /* Act. */
        ConfigurationScope symbolTable = createConfigurationSymbolTable(compilationUnit, MODEL_PATH);
        Optional<NestedConfigurationEntrySymbol> configurationEntryOpt = symbolTable.resolve("nested_with_no_entries", NestedConfigurationEntrySymbol.KIND);
        NestedConfigurationEntrySymbol configurationEntry = configurationEntryOpt.get();

        List<ConfigurationEntry> allConfigurationEntries = configurationEntry.getAllConfigurationEntries();

        /* Assert. */
        assertNotNull(allConfigurationEntries);
        assertTrue(allConfigurationEntries.isEmpty());
    }
}