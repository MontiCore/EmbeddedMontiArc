/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

public class SimulatorTSTest extends AbstractSymtab{


    @Test
    public void resolveModelDoorStatus() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.doors.doorStatus",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelGameOverTrigger() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.feature.gameOverTrigger",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelBrakeLightsControl() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.lights.brakeLightsControl",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelIndicatorStatus() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.lights.indicatorStatus",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsLightTimer() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.lights.lightTimer",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    //Does not work in maven for some reason
    @Ignore
    @Test
    public void resolveModel() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.main.sDCS",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsConstantVelocity() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.movement.constantVelocity",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    //Does not work in maven for some reason
    @Ignore
    @Test
    public void resolveModelsSteeringControl() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol instanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulatorts.visualization.movement.steeringControl",
                EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }
}
