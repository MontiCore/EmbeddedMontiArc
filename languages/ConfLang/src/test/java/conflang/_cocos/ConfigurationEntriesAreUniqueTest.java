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

public class ConfigurationEntriesAreUniqueTest extends AbstractTest {

    private static final String MODEL_PATH = "src/test/resources/conflang/_cocos";

    private ConfLangCoCoChecker checker = ConfLangCocoFactory.createCheckerWithCoCo(new ConfigurationEntriesAreUnique());

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
    public void simpleConfigurationEntryRepeated() {

        /* Arrange */
        ASTConfLangCompilationUnit compilationUnit = parse("src/test/resources/conflang/_cocos/SimpleConfigurationEntryRepeated.conf");
        ASTConfiguration configuration = compilationUnit.getConfiguration();

        /* Act */
        checker.checkAll(configuration);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, "repeated_entry")), new SourcePosition(5, 4))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void nestedConfigurationEntryRepeated() {

        /* Arrange */
        ASTConfLangCompilationUnit compilationUnit = parse("src/test/resources/conflang/_cocos/NestedConfigurationEntryRepeated.conf");
        ASTConfiguration configuration = compilationUnit.getConfiguration();

        /* Act */
        checker.checkAll(configuration);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, "repeated_nested_entry")), new SourcePosition(5, 4))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void configurationEntryRepeatedInSuperConfiguration() {

        /* Arrange */
        ASTConfLangCompilationUnit compilationUnit = parse("src/test/resources/conflang/_cocos/ConfigurationEntryRepeatedInSuperConfiguration.conf");
        createSymbolTable(compilationUnit, MODEL_PATH);
        ASTConfiguration configuration = compilationUnit.getConfiguration();

        /* Act */
        checker.checkAll(configuration);

        /* Assert */
        assertNotNull(configuration);
        assertEquals(2, Log.getErrorCount());
        Collection<Finding> expectedErrors = Lists.newArrayList(
                Finding.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, "repeated_entry")), new SourcePosition(4, 4)),
                Finding.error(ERROR_CODE_CL_01C.concat(String.format(ERROR_MSG_CL_01C, "repeated_nested_entry")), new SourcePosition(5, 4))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }
}