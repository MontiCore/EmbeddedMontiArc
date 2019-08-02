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

import java.time.Duration;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
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

        // puts physicalVehicle1 onto a position on the map that is known to be paved, physicalVehicle2 is already initialized with a position
        // that is known to be unpaved
        physicalVehicle1.putOnSurface(843, 236, 0.0);
        physicalVehicle1.computePhysics(Duration.ofMillis(1));
        physicalVehicle2.computePhysics(Duration.ofMillis(1));

        Assert.assertEquals(PhysicsEngine.calcFrictionCoefficient(StreetPavements.PAVED, WorldModel.getInstance().isItRaining()), physicalVehicle1.getVDM().getValue("mu_1"), 0);
        Assert.assertEquals(PhysicsEngine.calcFrictionCoefficient(StreetPavements.UNPAVED, WorldModel.getInstance().isItRaining()), physicalVehicle2.getVDM().getValue("mu_1"), 0);

        //physicalVehicle1.putOnSurface(200, 236, 0.0);

        System.out.println(physicalVehicle1);
        physicalVehicle1.computePhysics(Duration.ofMillis(4));
        System.out.println(physicalVehicle1);
    }

    public void mockSimulation() {
        RealVector expectedPosition = new ArrayRealVector(new double[]{843, 236, 0.0});
        RealVector setVelocity = new ArrayRealVector(new double[]{50.0, 0.0, 0.0});
        RealVector setAngularVelocity = new ArrayRealVector(new double[]{7.0, -2.5, 11.75});

        // Build vehicle
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        builder.setPosition(expectedPosition);
        builder.setVelocity(setVelocity);
        builder.setAngularVelocity(setAngularVelocity);
        ModelicaPhysicalVehicle physicalVehicle = (ModelicaPhysicalVehicle) builder.buildPhysicalVehicle();

        physicalVehicle.computePhysics(Duration.ofMillis(1));
        System.out.println(physicalVehicle);
        System.out.println(physicalVehicle.getVDM().getValue("mu_1"));
        physicalVehicle.computePhysics(Duration.ofMillis(2000));
        System.out.println(physicalVehicle);
        System.out.println(physicalVehicle.getVDM().getValue("mu_1"));
        // TODO: Print all values to use them in a graph
    }
}
