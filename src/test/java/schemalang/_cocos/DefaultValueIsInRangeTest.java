package schemalang._cocos;

import com.google.common.collect.Lists;
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

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.ERROR_CODE_SL_24C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_24C;

public class DefaultValueIsInRangeTest extends AbstractTest {

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
        checker.addCoCo(new DefaultValueIsInRange());
    }

    @Test
    public void defaultValueIsInRange() {
        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/" +
                "DefaultValueIsInRange.scm");
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void defaultValueIsNotInRange() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_cocos/" +
                "DefaultValueIsNotInRange.scm");

        /* Act */
        checker.checkAll(schemaLangDefinition);

        /* Assert */
        assertEquals(8, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "-3", "closed_interval_min", "[-2:2]")),
                        new SourcePosition(6, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "3", "closed_interval_max", "[-2:2]")),
                        new SourcePosition(7, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "-2", "open_interval_min", "(-2:2)")),
                        new SourcePosition(9, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "2", "open_interval_max", "(-2:2)")),
                        new SourcePosition(10, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "-3", "rightopen_interval_min", "[-2:2)")),
                        new SourcePosition(12, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "2", "rightopen_interval_max", "[-2:2)")),
                        new SourcePosition(13, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "-2", "leftopen_interval_min", "(-2:2]")),
                        new SourcePosition(15, 4)),
                Finding.error(ERROR_CODE_SL_24C.concat(String.format(ERROR_MSG_SL_24C,
                        "3", "leftopen_interval_max", "(-2:2]")),
                        new SourcePosition(16, 4))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }
}