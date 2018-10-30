/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;

public class SimulatorTSTest extends AbstractSymtab{


    @Test
    public void resolveModelDoorStatus() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.doors.doorStatus",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelGameOverTrigger() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.feature.gameOverTrigger",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelBrakeLightsControl() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.lights.brakeLightsControl",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelIndicatorStatus() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.lights.indicatorStatus",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsLightTimer() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.lights.lightTimer",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    //Does not work in maven for some reason
    @Ignore
    @Test
    public void resolveModel() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.main.sDCS",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    @Test
    public void resolveModelsConstantVelocity() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.movement.constantVelocity",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }

    //Does not work in maven for some reason
    @Ignore
    @Test
    public void resolveModelsSteeringControl() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol instanceSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulatorts.visualization.movement.steeringControl",
                ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(instanceSymbol);
    }
}