package conflang._parser;

import conflang.AbstractTest;
import conflang._symboltable.ConfigurationEntry;
import conflang._symboltable.ConfigurationSymbol;
import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.assertTrue;

public class ExtendsTest extends AbstractTest {

    @Test
    public void resolveInSuperConfiguration() {

        /* Arrange */
        ConfigurationSymbol configuration = parseAndGetConfLangConfigurationSymbol(
                "src/test/resources/conflang/_parser/extends/ExtendsConfiguration.conf",
                "src/test/resources/conflang/_parser/extends");

        /* Act */
        Optional<ConfigurationEntry> normalize = configuration.getConfigurationEntry("normalize");
        Optional<ConfigurationEntry> numEpoch = configuration.getConfigurationEntry("num_epoch");
        Optional<ConfigurationEntry> optimizer = configuration.getConfigurationEntry("optimizer");

        /* Assert */
        assertTrue(normalize.isPresent());
        assertTrue(numEpoch.isPresent());
        assertTrue(optimizer.isPresent());
    }
}