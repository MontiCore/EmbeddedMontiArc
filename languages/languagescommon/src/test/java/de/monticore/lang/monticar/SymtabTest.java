/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar;

import de.monticore.lang.monticar.stream._symboltable.ComponentStreamSymbol;
import de.monticore.lang.monticar.stream._symboltable.NamedStreamSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.util.Collection;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * Tests for symbol table of MontiArc.
 *
 */
public class SymtabTest extends AbstractSymtabTest {
    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
    }

    @Test
    public void testResolveComponentStreamSymbol() {
        Scope symTab = createSymTab("src/test/resources/nonunitstreams/streams");

        ComponentStreamSymbol comp = symTab.<ComponentStreamSymbol>resolve(
                "basicLibrary.And", ComponentStreamSymbol.KIND).orElse(null);
        assertNotNull(comp);
    }

    @Test
    public void testResolveNamedStreamSymbol() {
        Scope symTab = createSymTab("src/test/resources/nonunitstreams/streams");

        NamedStreamSymbol namedStreamSymbol = symTab.<NamedStreamSymbol>resolve(
                "advancedLibrary.RSFlipFlop.S", NamedStreamSymbol.KIND).orElse(null);
        assertNotNull(namedStreamSymbol);
    }

    @Ignore("ModelPath#resolveModel does not support loading a collection, which should be done when resolving many")
    @Test
    public void testResolveNamedManyStreamSymbol() {
        Scope symTab = createSymTab("src/test/resources/nonunitstreams/streams", "src/test/resources/nonunitstreams/streams2");

        Collection<NamedStreamSymbol> namedStreamSymbols = symTab.<NamedStreamSymbol>resolveMany(
                "advancedLibrary.RSFlipFlop.S", NamedStreamSymbol.KIND);
        assertEquals(2, namedStreamSymbols.size());
    }
}
