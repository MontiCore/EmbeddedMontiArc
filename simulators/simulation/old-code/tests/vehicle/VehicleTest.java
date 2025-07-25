/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.controller.interfaces.Bus;
import de.rwth.montisim.commons.controller.interfaces.FunctionBlockInterface;
import de.rwth.montisim.commons.simulation.PhysicalObject;
import de.rwth.montisim.commons.utils.Vec3;
import org.junit.AfterClass;

import static junit.framework.TestCase.assertEquals;
import static org.junit.Assert.*;

import org.junit.BeforeClass;
import org.junit.Test;

import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;
import de.rwth.montisim.simulation.environment.object.House;
import de.rwth.montisim.simulation.environment.util.VehicleType;
import de.rwth.montisim.simulation.util.Log;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * JUnit test for the Vehicle class
 */
public class VehicleTest {
    @Test
    public void computePhysicsTest() {
        Duration timeDiff = Duration.ofMillis(10);

        // 2 colliding vehicles
        PhysicalObject vehicle1 = createStandardVehicle(new MassPointPhysicalVehicleBuilder()).getPhysicalVehicle();
        PhysicalObject vehicle2 = createStandardVehicle(new MassPointPhysicalVehicleBuilder().setPosition(new Vec3(new double[]{1, 0, 0}))).getPhysicalVehicle();
        List<PhysicalObject> physicalObjects = new ArrayList<>();
        physicalObjects.add(vehicle2);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiff);
        assertTrue(vehicle1.getCollision() && vehicle2.getCollision());

        // 2 non colliding vehicles
        physicalObjects.clear();
        vehicle1 = createStandardVehicle(new MassPointPhysicalVehicleBuilder()).getPhysicalVehicle();
        vehicle2 = createStandardVehicle(new MassPointPhysicalVehicleBuilder().setPosition(new Vec3(new double[]{10, 0, 0}))).getPhysicalVehicle();
        physicalObjects.add(vehicle2);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiff);
        assertTrue(!vehicle1.getCollision() && !vehicle2.getCollision());

        // Vehicle colliding with non vehicle
        physicalObjects.clear();
        vehicle1 = createStandardVehicle(new MassPointPhysicalVehicleBuilder()).getPhysicalVehicle();
        vehicle2 = createStandardVehicle(new MassPointPhysicalVehicleBuilder().setPosition(new Vec3(new double[]{30, 0, 0}))).getPhysicalVehicle();
        House house = new House();
        house.setPosition(new Vec3(new double[]{1, 0, 0}));
        house.setWidth(10.0);
        house.setLength(10.0);
        house.setHeight(10.0);
        physicalObjects.add(vehicle2);
        physicalObjects.add(house);
        PhysicsEngine.computePhysics(vehicle1, physicalObjects, timeDiff);
        assertTrue(vehicle1.getCollision() && !vehicle2.getCollision());
    }

    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }

    @Test
    public void AutopilotBehaviorTest() throws Exception {
        /*       Masspoint       */

        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle.setVehicleType(VehicleType.ELECTRIC, 0);
        Vehicle vehicle2 = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle2.setVehicleType(VehicleType.ELECTRIC, 0.1);
        Vehicle vehicle3 = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle3.setVehicleType(VehicleType.ELECTRIC, 30);

        vehicle.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));
        vehicle2.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));
        vehicle3.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));


        assertTrue(vehicle.batteryProblem);
        assertTrue(vehicle2.batteryProblem);
        assertFalse(vehicle3.batteryProblem);
        assertTrue(vehicle.isGotoCharginstation());
        assertTrue(vehicle2.isGotoCharginstation());
        assertFalse(vehicle3.isGotoCharginstation());

        /*       Modelica       */

        Vehicle vehicle4 = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        vehicle4.setVehicleType(VehicleType.ELECTRIC, 0);
        Vehicle vehicle5 = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        vehicle5.setVehicleType(VehicleType.ELECTRIC, 0.1);
        Vehicle vehicle6 = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        vehicle6.setVehicleType(VehicleType.ELECTRIC, 30);


        vehicle4.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));
        vehicle5.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));
        vehicle6.getPhysicalVehicle().executeLoopIteration(Duration.ofMillis(10));

        assertTrue(vehicle4.batteryProblem);
        assertTrue(vehicle5.batteryProblem);
        assertFalse(vehicle6.batteryProblem);
        assertTrue(vehicle4.isGotoCharginstation());
        assertTrue(vehicle5.isGotoCharginstation());
        assertFalse(vehicle6.isGotoCharginstation());
    }
}
