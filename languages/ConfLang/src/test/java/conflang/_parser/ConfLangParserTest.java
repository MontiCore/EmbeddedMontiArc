/* (c) https://github.com/MontiCore/monticore */
package conflang._parser;

import conflang.AbstractTest;
import conflang._ast.ASTConfigurationEntry;
import conflang._ast.ASTConfLangCompilationUnit;
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
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/AllKindOfEntries.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void extendingConfiguration() {
        /* Act */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/Extending.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void supervisedLearningConfiguration() {
        /* Act */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/SupervisedLearning.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void ddpgConfiguration() {
        /* Act */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/DDPG.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void nullValueInConfiguration() {
        /* Act */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/NullValue.conf");

        /* Assert */
        assertNotNull(model);
    }

    //@Test
    public void siunitsInConfiguration() {
        /* Act */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/SIUnits.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void typelessAndComponentParametersInConfiguration() {
        /* Act */
        ASTConfLangCompilationUnit model = parse("src/test/resources/conflang/_parser/TypelessAndComponent.conf");

        /* Assert */
        assertNotNull(model);
    }

    @Test
    public void cnntrainConfigurations() {
        /* Act */
        parse("src/test/resources/conflang/_parser/AccuracyIgnoreLabel.conf");
        parse("src/test/resources/conflang/_parser/DefaultGAN.conf");
        parse("src/test/resources/conflang/_parser/Empty.conf");
        parse("src/test/resources/conflang/_parser/FlowNetEPE.conf");
        parse("src/test/resources/conflang/_parser/FullConfig.conf");
        parse("src/test/resources/conflang/_parser/FullConfig2.conf");
        parse("src/test/resources/conflang/_parser/FullConfig3.conf");
        parse("src/test/resources/conflang/_parser/FullConfig4.conf");
        parse("src/test/resources/conflang/_parser/ImageGAN.conf");
        parse("src/test/resources/conflang/_parser/InfoGAN.conf");
        parse("src/test/resources/conflang/_parser/Reinforcement.conf");
        parse("src/test/resources/conflang/_parser/Reinforcement2.conf");
        parse("src/test/resources/conflang/_parser/ReinforcementWithRosReward.conf");
        parse("src/test/resources/conflang/_parser/Simple1.conf");
        parse("src/test/resources/conflang/_parser/Simple2.conf");
        parse("src/test/resources/conflang/_parser/TD3.conf");
        parse("src/test/resources/conflang/_parser/Range.conf");

        /* Assert */
    }

    @Test
    public void simpleConfigurationEntry() throws RecognitionException, IOException {
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTConfigurationEntry> state = parser.parseConfigurationEntry(
                new StringReader("my_entry: 100"));
        assertFalse(parser.hasErrors());
        assertTrue(state.isPresent());
    }
}