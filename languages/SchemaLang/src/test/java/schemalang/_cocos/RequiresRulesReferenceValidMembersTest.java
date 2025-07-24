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
import static schemalang.ErrorCodes.ERROR_CODE_SL_20C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_20C;


public class RequiresRulesReferenceValidMembersTest extends AbstractTest {

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
        checker.addCoCo(new RequiresRulesReferenceValidMembers());
    }

    @Test
    public void undefinedPropertyOnLeftHandSide() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/UndefinedPropertyOnLeftHandSide.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(2, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, "noise_input", "noise_input requires num_epoch")),
                        new SourcePosition(8, 4)),
                Finding.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, "normalize", "normalize requires batch_size")),
                        new SourcePosition(9, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void undefinedPropertiesOnRightHandSide() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/UndefinedPropertyOnRightHandSide.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(3, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, "noise_input", "num_epoch requires noise_input, batch_size")),
                        new SourcePosition(8, 4)),
                Finding.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, "normalize", "batch_size requires normalize, optimizer")),
                        new SourcePosition(9, 4)),
                Finding.error(ERROR_CODE_SL_20C.concat(String.format(ERROR_MSG_SL_20C, "optimizer", "batch_size requires normalize, optimizer")),
                        new SourcePosition(9, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void validRequiresRules() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/ValidRequiresRules.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }
}