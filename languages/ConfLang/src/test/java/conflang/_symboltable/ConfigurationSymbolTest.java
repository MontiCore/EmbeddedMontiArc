package conflang._symboltable;

import conflang.AbstractTest;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.*;

public class ConfigurationSymbolTest extends AbstractTest {

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
    public void hasSimpleConfigurationEntry() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        boolean hasConfigurationEntry = configuration.containsConfigurationEntry("simple_entry");

        /* Assert. */
        assertTrue(hasConfigurationEntry);
    }

    @Test
    public void hasNestedConfigurationEntry() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        boolean hasConfigurationEntry = configuration.containsConfigurationEntry("nested_entry");

        /* Assert. */
        assertTrue(hasConfigurationEntry);
    }

    @Test
    public void hasNotConfigurationEntry() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        boolean hasConfigurationEntry = configuration.containsConfigurationEntry("not_existing_entry");

        /* Assert. */
        assertFalse(hasConfigurationEntry);
    }

    @Test
    public void getSimpleConfigurationEntry() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        Optional<ConfigurationEntry> configurationEntryOpt = configuration.getConfigurationEntry("simple_entry");

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());
        assertTrue(configurationEntryOpt.get().isOfSymbolKind(ConfigurationEntrySymbol.KIND));
        assertEquals("simple_entry", configurationEntryOpt.get().getName());
    }

    @Test
    public void getNestedConfigurationEntry() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        Optional<ConfigurationEntry> configurationEntryOpt = configuration.getConfigurationEntry("nested_entry");

        /* Assert. */
        assertTrue(configurationEntryOpt.isPresent());
        assertTrue(configurationEntryOpt.get().isOfSymbolKind(NestedConfigurationEntrySymbol.KIND));
        assertEquals("nested_entry", configurationEntryOpt.get().getName());
    }

    @Test
    public void getConfigurationEntryOfKindSimple() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        Optional<ConfigurationEntrySymbol> configurationEntrySymbolOpt = configuration.getConfigurationEntryOfKind("simple_entry", ConfigurationEntrySymbol.KIND);

        /* Assert */
        assertTrue(configurationEntrySymbolOpt.isPresent());

        ConfigurationEntrySymbol configurationEntrySymbol = configurationEntrySymbolOpt.get();
        assertEquals("simple_entry", configurationEntrySymbol.getName());
    }

    @Test
    public void getConfigurationEntryOfKindNested() {

        /* Arrange. */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol("src/test/resources/conflang/_symboltable/ConfLangConfigurationSymbol.conf",
                "src/test/resources/conflang/_symboltable");

        /* Act. */
        Optional<NestedConfigurationEntrySymbol> configurationEntrySymbolOpt = configuration.getConfigurationEntryOfKind("nested_entry", NestedConfigurationEntrySymbol.KIND);

        /* Assert */
        assertTrue(configurationEntrySymbolOpt.isPresent());

        NestedConfigurationEntrySymbol configurationEntrySymbol = configurationEntrySymbolOpt.get();
        assertEquals("nested_entry", configurationEntrySymbol.getName());
    }
}