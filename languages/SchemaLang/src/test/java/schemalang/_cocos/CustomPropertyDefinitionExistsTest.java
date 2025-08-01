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
import static schemalang.ErrorCodes.ERROR_CODE_SL_07C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_07C;

public class CustomPropertyDefinitionExistsTest extends AbstractTest {

    private SchemaLangCoCoChecker checker;

    private ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/_cocos"));

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
        checker = SchemaLangCocoFactory.createEmptyCoCoChecker();
        checker.addCoCo(new CustomPropertyDefinitionExists());
    }

    @Test
    public void complexDefinitionExists() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/ComplexDefinitionExists.scm");
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void complexDefinitionNotExists() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/ComplexDefinitionNotExists.scm");
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_07C.concat(String.format(ERROR_MSG_SL_07C, "eval_metric")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
        assertEquals(1, Log.getErrorCount());
    }
}