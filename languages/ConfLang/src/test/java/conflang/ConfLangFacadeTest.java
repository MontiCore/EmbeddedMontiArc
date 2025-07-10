package conflang;

import conflang._ast.ASTConfiguration;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.BeforeClass;

import java.nio.file.Paths;

import static org.junit.Assert.assertNotNull;

public class ConfLangFacadeTest {

    private ConfLangFacade facade;

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    //@Before
    public void setUp() throws RecognitionException {
        facade = ConfLangFacade.create(Paths.get("src/test/resources/conflang/parser"), "AllKindOfEntries.conf");
        Log.getFindings().clear();
    }


    //@Test
    public void allKindOfEntriesConfiguration() {
        /* Act */
        ASTConfiguration model = null; // parseModel("src/test/resources/conflang/parser/AllKindOfEntries.conf");

        /* Assert */
        assertNotNull(model);
    }
}