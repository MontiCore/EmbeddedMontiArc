package schemalang._symboltable;

import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;

public class SchemaLangScopeTest extends AbstractTest {

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() throws RecognitionException {
        Log.getFindings().clear();
    }

//    @Test
//    public void loadSuperSchema() {
//        /* Arrange */
//        ASTSchemaLangDefinition configuration = parseModel("src/test/resources/schemalang/symboltable/SchemaWithSuperSchema.scm");
//        ISchemaLangArtifactScope symbolTable = createSymbolTableFromAST(configuration);
//
//        /* Act */
//        ISchemaLangScope spannedScope = configuration.getSpannedScope();
//        spannedScope.getSchemaLangDefinitionSymbols();
//
//        /* Assert */
//    }
}