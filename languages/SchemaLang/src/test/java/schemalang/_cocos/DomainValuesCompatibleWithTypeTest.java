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
import static schemalang.ErrorCodes.ERROR_CODE_SL_06C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_06C;

public class DomainValuesCompatibleWithTypeTest extends AbstractTest {

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
        checker.addCoCo(new DomainValuesCompatibleWithType());
    }

    @Test
    public void domainValuesCompatible() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/DomainValuesCompatibleWithType.scm");
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void domainValuesNotCompatible() {

        /* Arrange */
        ASTSchemaDefinition schemaLangDefinition = parseSchemaDefinition("src/test/resources/schemalang/_cocos/" +
                "DomainValuesNotCompatibleWithType.scm");

        /* Act */
        checker.checkAll(schemaLangDefinition);

        /* Assert */
        assertEquals(2, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_06C.concat(String.format(ERROR_MSG_SL_06C,
                        "2.5", "num_epoch", "N")), new SourcePosition(6, 4)),
                Finding.error(ERROR_CODE_SL_06C.concat(String.format(ERROR_MSG_SL_06C,
                        "-2", "num_epoch", "N")), new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }
}