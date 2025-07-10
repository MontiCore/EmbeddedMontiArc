package schemalang._cocos;

import com.google.common.collect.Lists;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import schemalang.AbstractTest;
import schemalang._ast.ASTSchemaDefinition;

import java.util.Collection;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.ERROR_CODE_SL_04C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_04C;

public class DefaultValueCompatibleWithTypeTest extends AbstractTest {

    private SchemaLangCoCoChecker checker;

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
        checker = SchemaLangCocoFactory.createEmptyCoCoChecker();
        checker.addCoCo(new DefaultValueCompatibleWithType());
    }

    @Test
    public void initialValueCompatibleWithType() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_cocos/" +
                "InitialValueCompatibleWithType.scm");

        /* Act */
        checker.checkAll(schemaLangDefinition);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void initialValueNotCompatibleWithType() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_cocos/" +
                "InitialValueNotCompatibleWithType.scm");

        /* Act */
        checker.checkAll(schemaLangDefinition);

        /* Assert */
        assertEquals(12, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "(\"first\", 0.5, \"third\")", "string*")),
                        new SourcePosition(7, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "(0.0, 0, -11111)", "Z*")),
                        new SourcePosition(8, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "(\"my string\", 1.0E-6, 500)", "Q*")),
                        new SourcePosition(9, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "(\"true\", false, true)", "B*")),
                        new SourcePosition(10, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "(-1, 2, 3)", "N*")),
                        new SourcePosition(11, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "\"de.rwth.emadl.MyComponent\"", "component")),
                        new SourcePosition(14, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "-1", "N")),
                        new SourcePosition(17, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "0", "N1")),
                        new SourcePosition(18, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "-0.25", "Z")),
                        new SourcePosition(19, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "\"0.25\"", "Q")),
                        new SourcePosition(20, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "\"true\"", "B")),
                        new SourcePosition(21, 4)),
                Finding.error(ERROR_CODE_SL_04C.concat(String.format(ERROR_MSG_SL_04C, "no_string", "string")),
                        new SourcePosition(22, 4))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }
}