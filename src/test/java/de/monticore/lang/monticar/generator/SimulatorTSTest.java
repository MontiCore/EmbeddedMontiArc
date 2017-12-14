package de.monticore.lang.monticar.generator;

import org.junit.Assert;
import org.junit.Test;
import java.nio.file.*;
import de.monticore.lang.monticar.generator.cpp.resolver.ResolverFactory;
import de.monticore.lang.monticar.generator.cpp.resolver.Resolver;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

public class SimulatorTSTest {
    @Test
    public void resolveModel() {
        Path modelPath = Paths.get("src/test/resources/simulator-ts");

        ResolverFactory resolverFactory = new ResolverFactory(modelPath);
        Resolver resolver = resolverFactory.get();

        ExpandedComponentInstanceSymbol instanceSymbol =
                resolver.getExpandedComponentInstanceSymbol("montiarc.Constant_velocity").orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }
}
