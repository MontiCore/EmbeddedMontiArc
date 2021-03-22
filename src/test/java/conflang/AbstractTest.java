/* (c) https://github.com/MontiCore/monticore */
package conflang;

import conflang._ast.ASTConfLang;
import conflang._parser.ConfLangParser;
import org.antlr.v4.runtime.RecognitionException;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

import static org.junit.Assert.*;


/**
 * Provides some helpers for tests.
 */
abstract public class AbstractTest {

    /**
     * Parses a model and ensures that the root node is present.
     *
     * @param modelFile the full file name of the model.
     * @return the root of the parsed model.
     */
    protected ASTConfLang parseModel(String modelFile) {
        Path model = Paths.get(modelFile);
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTConfLang> configuration;
        try {
            configuration = parser.parse(model.toString());
            assertFalse(parser.hasErrors());
            assertTrue(configuration.isPresent());
            return configuration.get();
        } catch (RecognitionException | IOException e) {
            e.printStackTrace();
            fail("There was an exception when parsing the model " + modelFile + ": "
                    + e.getMessage());
        }
        return null;
    }
}
