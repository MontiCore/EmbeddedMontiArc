package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

public class SimulatorTSTest extends AbstractSymtab{

    @Test
    public void resolveModelDoorStatus() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("doors.doorStatus",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelGameOverTrigger() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("feature.gameOverTrigger",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelBrakeLightsControl() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("lights.brakeLightsControl",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelIndicatorStatus() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("lights.indicatorStatus",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsLightTimer() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("lights.lightTimer",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Ignore
    @Test
    public void resolveModel() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("main.sDCS",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsConstantVelocity() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("movement.constantVelocity",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsSteeringControl() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator-ts/visualization");

        ExpandedComponentInstanceSymbol instanceSymbol =
                symtab.<ExpandedComponentInstanceSymbol>resolve("movement.steeringControl",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }
}
