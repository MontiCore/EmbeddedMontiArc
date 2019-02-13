/**
 *
 * ******************************************************************************
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
package simulation.vehicle;

import org.junit.*;
import simulation.environment.WorldModel;
import simulation.environment.visualisationadapter.interfaces.EnvStreet.StreetPavements;


public class FrictionTest {
    @Test
    public  void testCoefficientValues() {
        double frictionCoefficientDry;
        double frictionCoefficientWet;

        for(StreetPavements pavement: StreetPavements.values()) {
            frictionCoefficientDry = PhysicsEngine.calcFrictionCoefficient(pavement, false);
            frictionCoefficientWet = PhysicsEngine.calcFrictionCoefficient(pavement, true);

            try {
                // test if values are between 0 and 1
                Assert.assertTrue(0 < frictionCoefficientDry && frictionCoefficientDry < 1 && 0 < frictionCoefficientWet && frictionCoefficientWet < 1);
            } catch (AssertionError e) {
                System.out.println("At least one friction coefficient for " + pavement.toString() + " is not correctly set. Check FrictionCoefficient.csv for entries with values >1 or <= 0");
                throw e;
            }

            try {
                // test if coefficient is smaller for a wet street
                Assert.assertTrue(frictionCoefficientWet < frictionCoefficientDry);
            } catch (AssertionError e) {
                System.out.println("The friction coefficient for a dry street with pavement " + pavement.toString() + " is set to be smaller than the value for a wet pavement.");
                throw e;
            }
        }
    }

    @Test
    public void testPavementForSurface() {
        ModelicaPhysicalVehicle physicalVehicle1 = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        ModelicaPhysicalVehicle physicalVehicle2 = (ModelicaPhysicalVehicle) new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();

        // puts physicalVehicle1 onto a position on the map that is known to be paved, physicalVehicle2 is initialized with such a position
        physicalVehicle1.putOnSurface(843, 236, 0.0);
        physicalVehicle1.computePhysics(1);
        physicalVehicle2.computePhysics(1);

        Assert.assertEquals(PhysicsEngine.calcFrictionCoefficient(StreetPavements.PAVED, WorldModel.getInstance().isItRaining()), physicalVehicle1.getVDM().getValue("mu_1"), 0);
        Assert.assertEquals(PhysicsEngine.calcFrictionCoefficient(StreetPavements.UNPAVED, WorldModel.getInstance().isItRaining()), physicalVehicle2.getVDM().getValue("mu_1"), 0);
    }
}
