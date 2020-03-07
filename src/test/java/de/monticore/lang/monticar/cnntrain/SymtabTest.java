/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain;

import de.monticore.lang.monticar.cnntrain._parser.CNNTrainParser;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigurationSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;


public class SymtabTest extends AbstractSymtabTest {

    @Test
    public void testParsing() throws Exception {
        CNNTrainParser parser = new CNNTrainParser();
        assertTrue(parser.parse("src/test/resources/valid_tests/SimpleConfig1.cnnt").isPresent());
    }

    @Test
    public void testSymtab(){
        Scope symTab = createSymTab("src/test/resources/valid_tests/");
        CNNTrainCompilationUnitSymbol a = symTab.<CNNTrainCompilationUnitSymbol>resolve(
                "SimpleConfig2",
                CNNTrainCompilationUnitSymbol.KIND).orElse(null);

        assertNotNull(a);

    }
}
