package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.monticar.generator.middleware.clustering.FlattenArchitecture;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import static org.junit.Assert.*;
import static org.junit.Assert.assertTrue;

public class FlattenAlgorithmTest {

    public static final String TEST_PATH = "src/test/resources/";

    @Test
    public void testFlattenAlgorithm1(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.overallSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol);
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(10, subComponents.size());
        assertEquals(20, connectors.size());
    }

    @Test
    public void testFlattenAlgorithm1ShortNames(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.overallSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol, new HashMap<>());
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(10, subComponents.size());
        assertEquals(20, connectors.size());

        EMAComponentInstanceSymbol testComponentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.overallSystemShortNameTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(testComponentInstanceSymbol);
        assertEqualEMAComponentInstanceSymbol(testComponentInstanceSymbol, newComponentInstanceSymbol);
    }



    @Test
    public void testFlattenAlgorithm2(){
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol);
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(20, subComponents.size());
        assertEquals(40, connectors.size());
    }

    @Test
    public void testFlattenAlgorithm2ShortNames() {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol, new HashMap<>());
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(20, subComponents.size());
        assertEquals(40, connectors.size());

        EMAComponentInstanceSymbol testComponentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystemShortNameTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(testComponentInstanceSymbol);
        assertEqualEMAComponentInstanceSymbol(testComponentInstanceSymbol, newComponentInstanceSymbol);
    }

    @Test
    public void testFlattenAlgorithmWithLevels() {
        TaggingResolver taggingResolver = AbstractSymtabTest.createSymTabAndTaggingResolver(TEST_PATH);

        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver
                .<EMAComponentInstanceSymbol>resolve("lab.spanningSystem", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        EMAComponentInstanceSymbol newComponentInstanceSymbol = FlattenArchitecture
                .flattenArchitecture(componentInstanceSymbol, new HashMap<>(), 2);
        assertNotNull(newComponentInstanceSymbol);
        Collection<EMAComponentInstanceSymbol> subComponents = newComponentInstanceSymbol.getSubComponents();
        Collection<EMAConnectorInstanceSymbol> connectors = newComponentInstanceSymbol.getConnectorInstances();
        assertEquals(4, subComponents.size());
        assertEquals(24, connectors.size());
    }

    private static void assertEqualEMAComponentInstanceSymbol(EMAComponentInstanceSymbol symbol1, EMAComponentInstanceSymbol symbol2) {
        List<String> symbol1Subcomponents = new ArrayList();
        List<String> symbol2Subcomponents = new ArrayList();

        List<String> symbol1Ports = new ArrayList();
        List<String> symbol2Ports = new ArrayList();

        List<String> symbol1Connectors = new ArrayList();
        List<String> symbol2Connectors = new ArrayList();

        for (EMAComponentInstanceSymbol s : symbol1.getSubComponents()) {
            symbol1Subcomponents.add(s.getName());
        }

        for (EMAComponentInstanceSymbol s : symbol2.getSubComponents()) {
            symbol2Subcomponents.add(s.getName());
        }

        for (EMAConnectorInstanceSymbol c : symbol1.getConnectorInstances()) {
            symbol1Connectors.add(c.getName());
        }

        for (EMAConnectorInstanceSymbol c : symbol2.getConnectorInstances()) {
            symbol2Connectors.add(c.getName());
        }

        for (EMAPortInstanceSymbol p : symbol1.getPortInstanceList()) {
            symbol1Ports.add(p.getName());
        }

        for (EMAPortInstanceSymbol p : symbol2.getPortInstanceList()) {
            symbol2Ports.add(p.getName());
        }

        assertTrue(symbol1Ports.containsAll(symbol2Ports));
        assertTrue(symbol2Ports.containsAll(symbol1Ports));

        assertTrue(symbol1Connectors.containsAll(symbol2Connectors));
        assertTrue(symbol2Connectors.containsAll(symbol1Connectors));

        assertTrue(symbol1Subcomponents.containsAll(symbol2Subcomponents));
        assertTrue(symbol2Subcomponents.containsAll(symbol1Subcomponents));
    }
}
