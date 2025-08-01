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
import schemalang._symboltable.SchemaLangArtifactScope;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static schemalang.ErrorCodes.ERROR_CODE_SL_03C;
import static schemalang.ErrorCodes.ERROR_MSG_SL_03C;


public class PropertyDefinitionsAreUniqueTest extends AbstractTest {

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
        checker.addCoCo(new PropertyDefinitionsAreUnique());
    }

    @Test
    public void schemaWithRepeatedSchemaMember() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/SchemaWithRepeatedSchemaMember.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_03C.concat(String.format(ERROR_MSG_SL_03C, "num_epoch", "SchemaWithRepeatedSchemaMember")),
                        new SourcePosition(7, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void schemaMemberAlreadyDefinedInSuperSchema() {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/_cocos/SchemaWithRepeatedSchemaMemberInSuperSchema.scm");

        // Must be called! Otherwise the linking between AST nodes and symbols will not work.
        SchemaLangArtifactScope symbolTable = createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning(ERROR_CODE_SL_03C.concat(String.format(ERROR_MSG_SL_03C, "batch_size", "SchemaWithRepeatedSchemaMemberInSuperSchema")),
                        new SourcePosition(6, 4)));
        assertErrors(expectedErrors, Log.getFindings());
    }
}