/* (c) https://github.com/MontiCore/monticore */
package conflang.cocos;

import conflang.AbstractTest;
import conflang._ast.ASTConfLang;
import conflang._cocos.ConfLangCoCoChecker;
import de.se_rwth.commons.SourcePosition;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.Collection;
import java.util.Collections;

import static de.monticore.cocos.helper.Assert.assertErrors;

public class ConfLangCoCosTest extends AbstractTest {

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Before
    public void setUp() throws RecognitionException {
        Log.getFindings().clear();
    }

    @Test
    public void configurationNameStartWithLowerLetter() {
        /* Arrange */
        ASTConfLang configuration = parseModel("src/test/resources/conflang/cocos/ConfigurationNameStartsWithLowerLetter.conf");

        /* Act */
        ConfLangCoCos cocos = new ConfLangCoCos();
        ConfLangCoCoChecker checker = cocos.getCheckerForAllCoCos();
        checker.checkAll(configuration);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning("0xB4002 Configuration name 'myConfiguration' must start with a capital letter.",
                        new SourcePosition(3, 0)));
        assertErrors(expectedErrors, Log.getFindings());
    }

    @Test
    public void configurationEntryRepeated() {
        /* Arrange */
        ASTConfLang configuration = parseModel("src/test/resources/conflang/cocos/ConfigurationEntryRepeated.conf");

        /* Act */
        ConfLangCoCos cocos = new ConfLangCoCos();
        ConfLangCoCoChecker checker = cocos.getCheckerForAllCoCos();
        checker.checkAll(configuration);

        /* Assert */
        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.warning("0xB4002 Configuration entries must not be repeated.",
                        new SourcePosition(3, 0)));
        assertErrors(expectedErrors, Log.getFindings());
    }
}