/* (c) https://github.com/MontiCore/monticore */
package conflang._cocos;

import com.google.common.collect.Lists;
import conflang.AbstractTest;
import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfiguration;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Collection;

import static conflang.ErrorCodes.ERROR_CODE_CL_01C;
import static conflang.ErrorCodes.ERROR_MSG_CL_01C;
import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class ConfigurationEntriesInNestedEntryAreUniqueTest extends AbstractTest {

    private static final String MODEL_PATH = "src/test/resources/conflang/_cocos";

    private ConfLangCoCoChecker checker = ConfLangCocoFactory.createCheckerWithCoCo(new ConfigurationEntriesInNestedEntryAreUnique());

    @BeforeClass
    public static void init() {
        Log.init();
        Log.enableFailQuick(false);
    }

    @Before
    public void before() {
        Log.clearFindings();
    }

    @Test
    public void configurationEntriesRepeatedInNestedEntry() {

        /* Arrange */
        ASTConfLangCompilationUnit compilationUnit = parse("src/test/resources/conflang/_cocos/ConfigurationEntriesRepeatedInNestedEntry.conf");
        ASTConfiguration configuration = compilationUnit.getConfiguration();

        /* Act */
        checker.checkAll(configuration);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(2, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, "repeated_entry")), new SourcePosition(7, 8)),
                Finding.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, "nested_entry")), new SourcePosition(13, 8))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void configurationEntriesNotRepeatedInNestedEntry() {

        /* Arrange */
        ASTConfLangCompilationUnit compilationUnit = parse("src/test/resources/conflang/_cocos/ConfigurationEntriesNotRepeatedInNestedEntry.conf");
        ASTConfiguration configuration = compilationUnit.getConfiguration();

        /* Act */
        checker.checkAll(configuration);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(0, Log.getErrorCount());
    }
}