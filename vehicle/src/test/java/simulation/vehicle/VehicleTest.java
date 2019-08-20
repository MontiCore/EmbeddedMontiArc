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

import commons.simulation.PhysicalObject;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.BeforeClass;
import org.junit.Test;
import simulation.environment.object.House;
import simulation.util.Log;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;

/**
 * JUnit test for the Vehicle class
 */
public class VehicleTest {
    @Test
    public void computePhysicsTest() {
        Duration timeDiff = Duration.ofMillis(10);

        // 2 colliding vehicles
        PhysicalObject vehicle1 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        PhysicalObject vehicle2 = new MassPointPhysicalVehicleBuilder().setPosition(new ArrayRealVector(new double[] {1, 0, 0})).buildPhysicalVehicle();
        List<PhysicalObject> physicalObjects = new ArrayList<>();
        physicalObjects.add(vehicle2);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiff);
        assertTrue(vehicle1.getCollision() && vehicle2.getCollision());

        // 2 non colliding vehicles
        physicalObjects.clear();
        vehicle1 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        vehicle2 = new MassPointPhysicalVehicleBuilder().setPosition(new ArrayRealVector(new double[] {10, 0, 0})).buildPhysicalVehicle();
        physicalObjects.add(vehicle2);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiff);
        assertTrue(!vehicle1.getCollision() && !vehicle2.getCollision());

        // Vehicle colliding with non vehicle
        physicalObjects.clear();
        vehicle1 = new MassPointPhysicalVehicleBuilder().buildPhysicalVehicle();
        vehicle2 = new MassPointPhysicalVehicleBuilder().setPosition(new ArrayRealVector(new double[] {30, 0, 0})).buildPhysicalVehicle();
        House house = new House();
        house.setPosition(new ArrayRealVector(new double[] {1, 0, 0}));
        house.setWidth(10.0);
        house.setLength(10.0);
        house.setHeight(10.0);
        physicalObjects.add(vehicle2);
        physicalObjects.add(house);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiff);
        assertTrue(vehicle1.getCollision() && !vehicle2.getCollision());
    }
}