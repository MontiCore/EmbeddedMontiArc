/* (c) https://github.com/MontiCore/monticore */
package conflang.cocos;

import conflang.AbstractTest;
import conflang._ast.ASTConfLang;
import de.monticore.cocos.helper.Assert;
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

import static org.junit.Assert.*;

public class ConfigurationNameStartsWithCapitalLetterTest extends AbstractTest {

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
    public void valid() {
        /* Arrange */
        ASTConfLang configuration = parseModel("src/test/resources/conflang/cocos/ConfigurationNameStartsWithCapitalLetter.conf");

        /* Act */
        ConfigurationNameStartsWithCapitalLetter coco = new ConfigurationNameStartsWithCapitalLetter();
        coco.check(configuration);

        /* Assert */
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void invalid() {
        /* Arrange */
        ASTConfLang configuration = parseModel("src/test/resources/conflang/cocos/ConfigurationNameStartsWithLowerLetter.conf");

        /* Act */
        ConfigurationNameStartsWithCapitalLetter coco = new ConfigurationNameStartsWithCapitalLetter();
        coco.check(configuration);

        /* Assert */
        assertFalse(Log.getFindings().isEmpty());
        assertEquals(1, Log.getErrorCount());

        Collection<Finding> expectedErrors = Collections.singletonList(
                Finding.error("0xB4002 Configuration name 'myConfiguration' must start with a capital letter.",
                        new SourcePosition(3, 0)));
        Assert.assertErrors(expectedErrors, Log.getFindings());
    }
}
