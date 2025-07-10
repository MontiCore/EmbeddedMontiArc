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
import schemalang.validation.exception.SchemaLangException;

import java.nio.file.Paths;
import java.util.Collection;

import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static schemalang.ErrorCodes.*;

public class ComponentsDoNotConflictWithReferenceModelTest extends AbstractTest {

    private SchemaLangCoCoChecker checker;

    private ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/validation/referencemodels"));

    @BeforeClass
    public static void beforeClass() {
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
        checker = new SchemaLangCoCoChecker();
        checker.addCoCo(new ComponentsDoNotConflictWithReferenceModel());
    }

    @Test
    public void criticComponentDefinedInSchema() throws SchemaLangException {

        /* Arrange */
        ASTSchemaLangCompilationUnit schemaLangCompilationUnit = parse("src/test/resources/schemalang/validation" +
                "/referencemodels/CriticComponentDefinedInSchema.scm");
        createSymbolTable2(schemaLangCompilationUnit, modelPath);

        /* Act */
        checker.checkAll(schemaLangCompilationUnit);

        /* Assert */
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_TA_17C.concat(String.format(ERROR_MSG_TA_17C, "critic",
                        "CriticComponentDefinedInSchema", "ddpg9.DDPG")), new SourcePosition(5, 4))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }
}