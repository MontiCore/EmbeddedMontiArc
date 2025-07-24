/* (c) https://github.com/MontiCore/monticore */
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
import schemalang._ast.ASTSchemaLangCompilationUnit;

import java.nio.file.Paths;
import java.util.Collection;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.*;


public class RequiresRulesAreUniqueTest extends AbstractTest {

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
        checker.addCoCo(new RequiresRulesAreUnique());
    }

    @Test
    public void repeatedRequiresRules() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/RepeatedRequiresRules.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(2, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_19C.concat(String.format(ERROR_MSG_SL_19C, "noise_input requires num_epoch", "RepeatedRequiresRules")),
                        new SourcePosition(6, 4)),
                Finding.error(ERROR_CODE_SL_19C.concat(String.format(ERROR_MSG_SL_19C, "normalize requires batch_size", "RepeatedRequiresRules")),
                        new SourcePosition(8, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void repeatedRequiresRulesInSubSchema() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/RepeatedRequiresRulesInSubSchema.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(2, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_19C.concat(String.format(ERROR_MSG_SL_19C, "noise_input requires num_epoch", "RepeatedRequiresRulesInSubSchema")),
                        new SourcePosition(5, 4)),
                Finding.error(ERROR_CODE_SL_19C.concat(String.format(ERROR_MSG_SL_19C, "normalize requires batch_size", "RepeatedRequiresRulesInSubSchema")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }
}