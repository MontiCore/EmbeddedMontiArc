/* (c) https://github.com/MontiCore/monticore */
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

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.*;

public class CustomPropertyDefinitionOverrideOnlyInSubSchemaTest extends AbstractTest {

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
        checker.addCoCo(new CustomPropertyDefinitionOverrideOnlyInSubSchema());
    }

    @Test
    public void overrideWithPropertyInSuperSchema() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/OverrideWithPropertyInSuperSchema.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(0, Log.getErrorCount());
    }

    @Test
    public void overrideWithoutSuperSchema() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/OverrideWithoutSuperSchema.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.error(ERROR_CODE_SL_17C.concat(String.format(ERROR_MSG_SL_17C, "strategy", "OverrideWithoutSuperSchema")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void overrideWithoutPropertyInSuperSchema() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/OverrideWithoutPropertyInSuperSchema.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.error(ERROR_CODE_SL_18C.concat(String.format(ERROR_MSG_SL_18C, "strategy")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }
}