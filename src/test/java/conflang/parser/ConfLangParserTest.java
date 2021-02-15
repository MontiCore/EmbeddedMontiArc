/* (c) https://github.com/MontiCore/monticore */
package conflang.parser;

import conflang.AbstractTest;
import conflang._ast.ASTConfLang;
import conflang._ast.ASTSimpleConfigurationEntry;
import conflang._parser.ConfLangParser;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Test;

import java.io.IOException;
import java.io.StringReader;
import java.util.Optional;

import static org.junit.Assert.*;

public class ConfLangParserTest extends AbstractTest {

    @Test
    public void allKindOfEntriesConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/AllKindOfEntries.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void extendingConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/Extending.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void supervisedLearningConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/SupervisedLearning.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void simpleConfigurationEntry() throws RecognitionException, IOException {
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTSimpleConfigurationEntry> state = parser.parseSimpleConfigurationEntry(
                new StringReader("my_entry = 100"));
        assertFalse(parser.hasErrors());
        assertTrue(state.isPresent());
    }
}
