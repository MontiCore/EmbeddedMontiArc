package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Assert;
import org.junit.Test;
import java.nio.file.*;
import de.monticore.lang.monticar.generator.cpp.resolver.ResolverFactory;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

public class SimulatorTSTest extends AbstractSymtab{
    @Test
    public void resolveModel() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("montiarc.constant_velocity",ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }
}
