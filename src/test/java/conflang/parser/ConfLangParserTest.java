/* (c) https://github.com/MontiCore/monticore */
package conflang.parser;

import conflang.AbstractTest;
import conflang._ast.ASTConfLang;
import conflang._ast.ASTSimpleConfigurationEntry;
import conflang._parser.ConfLangParser;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.Test;

import java.io.IOException;
import java.io.StringReader;
import java.util.Optional;

import static org.junit.Assert.*;

public class ConfLangParserTest extends AbstractTest {

    @Test
    public void allKindOfEntriesConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/AllKindOfEntries.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void extendingConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/Extending.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void supervisedLearningConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/SupervisedLearning.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void ddpgConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/DDPG.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void nullValueInConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/NullValue.conf");

        /* Assert */
        assertNotNull(model);
    }

    //@Test
    public void siunitsInConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/SIUnits.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void typelessAndComponentParametersInConfiguration() {
        /* Act */
        ASTConfLang model = parseModel("src/test/resources/conflang/parser/TypelessAndComponent.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void cnntrainConfigurations() {
        /* Act */
        parseModel("src/test/resources/conflang/parser/AccuracyIgnoreLabel.conf");
        parseModel("src/test/resources/conflang/parser/DefaultGAN.conf");
        parseModel("src/test/resources/conflang/parser/Empty.conf");
        parseModel("src/test/resources/conflang/parser/FlowNetEPE.conf");
        parseModel("src/test/resources/conflang/parser/FullConfig.conf");
        parseModel("src/test/resources/conflang/parser/FullConfig2.conf");
        parseModel("src/test/resources/conflang/parser/FullConfig3.conf");
        parseModel("src/test/resources/conflang/parser/FullConfig4.conf");
        parseModel("src/test/resources/conflang/parser/ImageGAN.conf");
        parseModel("src/test/resources/conflang/parser/InfoGAN.conf");
        parseModel("src/test/resources/conflang/parser/Reinforcement.conf");
        parseModel("src/test/resources/conflang/parser/Reinforcement2.conf");
        parseModel("src/test/resources/conflang/parser/ReinforcementWithRosReward.conf");
        parseModel("src/test/resources/conflang/parser/Simple1.conf");
        parseModel("src/test/resources/conflang/parser/Simple2.conf");
        parseModel("src/test/resources/conflang/parser/TD3.conf");

        /* Assert */
    }

    @Test
    public void simpleConfigurationEntry() throws RecognitionException, IOException {
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTSimpleConfigurationEntry> state = parser.parseSimpleConfigurationEntry(
                new StringReader("my_entry = 100"));
        assertFalse(parser.hasErrors());
        assertTrue(state.isPresent());
    }
}
