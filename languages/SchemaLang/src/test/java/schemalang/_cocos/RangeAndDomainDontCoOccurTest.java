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

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.*;

public class RangeAndDomainDontCoOccurTest extends AbstractTest {

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
        checker.addCoCo(new RangeAndDomainDontCoOccur());
    }

    @Test
    public void rangeAndDomainCoOccur() {
        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/" +
                "RangeAndDomainCoOccur.scm");

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_23C.concat(String.format(ERROR_MSG_SL_23C, "num_epoch")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void RangeAndDomainDontCoOccur() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_cocos/" +
                "RangeAndDomainDontCoOccur.scm");

        /* Act */
        checker.checkAll(schemaLangDefinition);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }
}