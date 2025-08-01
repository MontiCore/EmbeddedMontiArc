package schemalang._cocos;

import de.monticore.io.paths.ModelPath;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaDefinition;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._symboltable.SchemaLangArtifactScope;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.*;

public class SchemaEnumsOnlyOneAllowedTest extends AbstractTest {

    private SchemaLangCoCoChecker checker;

    private ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/_cocos"));

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
        checker = SchemaLangCocoFactory.createEmptyCoCoChecker();
        checker.addCoCo(new SchemaEnumsOnlyOneAllowed());
    }

    @Test
    public void multipleSchemaEnumsNotAllowed() {
        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/" +
                "MultipleSchemaEnumsNotAllowed.scm");
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_22C.concat(ERROR_MSG_SL_22C),
                        new SourcePosition(4, 0)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void onlyOneSchemaEnumDefined() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_cocos/OnlyOneSchemaEnumDefined.scm");

        /* Act */
        checker.checkAll(schemaLangDefinition);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }
}