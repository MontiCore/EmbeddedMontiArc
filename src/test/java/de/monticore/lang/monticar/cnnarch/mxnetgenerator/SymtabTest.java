/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.mxnetgenerator;

import de.monticore.lang.monticar.cnnarch._parser.CNNArchParser;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class SymtabTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testParsing() throws Exception {
        CNNArchParser parser = new CNNArchParser();
        assertTrue(parser.parse("src/test/resources/architectures/Alexnet.cnna").isPresent());
    }

    @Ignore
    @Test
    public void testAlexnet(){
        Scope symTab = createSymTab("src/test/resources/architectures");
        CNNArchCompilationUnitSymbol a = symTab.<CNNArchCompilationUnitSymbol>resolve(
                "Alexnet",
                CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull(a);
        a.resolve();
        a.getArchitecture().getStreams().get(0).getOutputTypes();
    }

    @Ignore
    @Test
    public void testResNeXt(){
        Scope symTab = createSymTab("src/test/resources/architectures");
        CNNArchCompilationUnitSymbol a = symTab.<CNNArchCompilationUnitSymbol>resolve(
                "ResNeXt50",
                CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull(a);
        a.resolve();
        a.getArchitecture().getStreams().get(0).getOutputTypes();
    }

    @Ignore
    @Test
    public void test3(){
        Scope symTab = createSymTab("src/test/resources/valid_tests");
        CNNArchCompilationUnitSymbol a = symTab.<CNNArchCompilationUnitSymbol>resolve(
                "MultipleOutputs",
                CNNArchCompilationUnitSymbol.KIND).orElse(null);
        assertNotNull(a);
        a.resolve();
        a.getArchitecture().getStreams().get(0).getOutputTypes();
    }

}
