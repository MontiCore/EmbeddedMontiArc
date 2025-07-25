/* (c) https://github.com/MontiCore/monticore */
package conflang._cocos;

import conflang.AbstractTest;
import conflang._ast.ASTConfLangCompilationUnit;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Collection;
import java.util.Collections;

import static conflang.ErrorCodes.*;
import static de.monticore.cocos.helper.Assert.assertErrors;
import static org.junit.Assert.*;

public class ConfigurationNameStartsWithCapitalLetterTest extends AbstractTest {

    private ConfLangCoCoChecker checker = ConfLangCocoFactory.createCheckerWithCoCo(new ConfigurationNameStartsWithCapitalLetter());

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() throws RecognitionException {
        LogStub.init();
        Log.getFindings().clear();
    }

    @Test
    public void configurationNameStartsWithCapitalLetter() {
        /* Arrange */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_cocos/ConfigurationNameStartsWithCapitalLetter.conf");

        /* Act */
        checker.checkAll(model.getConfiguration());

        /* Assert */
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void configurationNameStartsWithLowerLetter() {
        /* Arrange */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_cocos/configurationNameStartsWithLowerLetter.conf");

        /* Act */
        checker.checkAll(model.getConfiguration());

        /* Assert */
        assertFalse(Log.getFindings().isEmpty());
        assertEquals(1, Log.getErrorCount());
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.error(ERROR_CODE_CL_02C.concat(String.format(ERROR_MSG_CL_02C, "configurationNameStartsWithLowerLetter")), new SourcePosition(3, 0))
        );
        assertErrors(expectedErrors, Log.getFindings());
    }
}