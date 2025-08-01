package schemalang._cocos;

import de.monticore.io.paths.ModelPath;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._symboltable.SchemaLangArtifactScope;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.ERROR_CODE_SL_08C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_08C;

public class FileAndSchemaNameAreEqualTest extends AbstractTest {

    private SchemaLangCoCoChecker checker;

    private ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/_cocos"));

    @BeforeClass
    public static void beforeClass() {
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
        checker = new SchemaLangCoCoChecker();
        checker.addCoCo(new FileAndSchemaNameAreEqual());
    }

    @Test
    public void fileNameNotEqual() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/FileNameNotEqual.scm");
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_08C.concat(ERROR_MSG_SL_08C), new SourcePosition(3, 0)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void fileNameEqual() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/FileNameEqual.scm");
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }
}