package schemalang._cocos;

import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaLangCompilationUnit;
import schemalang._symboltable.SchemaLangArtifactScope;

import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.*;

public class CustomPropertyHasDifferentNameTest extends AbstractTest {

    private SchemaLangCoCoChecker checker;

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
        checker = SchemaLangCocoFactory.createEmptyCoCoChecker();
        checker.addCoCo(new CustomPropertyHasDifferentName());
    }

    @Test
    public void customPropertyHasDifferentName() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/CustomPropertyHasDifferentName.scm");

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void customPropertyHasNotDifferentName() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/CustomPropertyHasNotDifferentName.scm");

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_09C.concat(String.format(ERROR_MSG_SL_09C, "eval_metric")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }
}