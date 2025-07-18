/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import de.rwth.montisim.commons.utils.Vec3;
import org.junit.*;

import static org.junit.Assert.*;

import de.rwth.montisim.simulation.environment.util.ChargingProcess;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.util.VehicleType;
import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;

import java.time.Instant;


/**
 * Charging Station Test Class
 *
 * @version 1.0
 * @since 2019-09-17
 */
public class ChargingStationTest {

    private Vehicle createStandardVehicle(PhysicalVehicleBuilder physicalVehicleBuilder) {
        EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
        EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
        InstantBus bus = new InstantBus(eeSimulator);
        eeVehicleBuilder.createAllSensorsNActuators(bus);
        return new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
    }

    @Test
    public void startCharging() throws Exception {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation = new ChargingStation();

        // Test for non-electric Vehicle
        PhysicalVehicle vehicleNotElectric = new ModelicaPhysicalVehicle();
        assertFalse(chargingStation.startCharging(vehicleNotElectric));

        // Test for car that is not standing near CS
        Vehicle vehicleNotNear = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicleNotNear.setVehicleType(VehicleType.ELECTRIC, 25);
        vehicleNotNear.getPhysicalVehicle().setPosition(new Vec3(new double[]{10, 5, 0}));
        assertFalse(chargingStation.startCharging(vehicleNotNear.getPhysicalVehicle()));

        // Test for electric car standing near CS
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle.setVehicleType(VehicleType.ELECTRIC, 10);
        assertTrue(chargingStation.startCharging(vehicle.getPhysicalVehicle()));
        assertTrue(chargingStation.getCarObjects().size() == 1);

        // Test for occupied CS
        Vehicle vehicle2 = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle2.setVehicleType(VehicleType.ELECTRIC, 10);
        assertFalse(chargingStation.startCharging(vehicle2.getPhysicalVehicle()));
    }

    @Test
    public void stopCharging() throws Exception {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation = new ChargingStation();

        // Create and charge vehicle
        Vehicle vehicle = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicle.setVehicleType(VehicleType.ELECTRIC, 50);
        chargingStation.startCharging(vehicle.getPhysicalVehicle());

        // Test for vehicle that is not charged at the CS
        Vehicle vehicleNotCharged = createStandardVehicle(new MassPointPhysicalVehicleBuilder());
        vehicleNotCharged.setVehicleType(VehicleType.ELECTRIC, 80);
        assertFalse(chargingStation.stopCharging(vehicleNotCharged.getPhysicalVehicle()));

        // Stop charging
        assertTrue(chargingStation.stopCharging(vehicle.getPhysicalVehicle()));
        assertTrue(chargingStation.getCarObjects().isEmpty());
        assertFalse(chargingStation.stopCharging(vehicle.getPhysicalVehicle()));
    }

    @Test
    public void isOccupied() throws Exception {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation1 = new ChargingStation();
        // Create ChargingStation with capacity=3
        ChargingStation chargingStation2 = new ChargingStation();
        chargingStation2.setCapacity(3);

        // Test ChargingStation not occupied
        assertFalse(chargingStation1.isOccupied());
        assertFalse(chargingStation2.isOccupied());

        // Create Vehicle
        Vehicle vehicle1 = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        vehicle1.setVehicleType(VehicleType.ELECTRIC, 10);
        Vehicle vehicle2 = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        vehicle2.setVehicleType(VehicleType.ELECTRIC, 20);
        Vehicle vehicle3 = createStandardVehicle(new ModelicaPhysicalVehicleBuilder());
        vehicle3.setVehicleType(VehicleType.ELECTRIC, 30);

        // Test ChargingStation occupied for ChargingStation1
        chargingStation1.startCharging(vehicle1.getPhysicalVehicle());
        assertTrue(chargingStation1.isOccupied());

        // Test ChargingStation occupied for ChargingStation2
        chargingStation2.startCharging(vehicle1.getPhysicalVehicle());
        assertFalse(chargingStation2.isOccupied());
        chargingStation2.startCharging(vehicle2.getPhysicalVehicle());
        assertFalse(chargingStation2.isOccupied());
        chargingStation2.startCharging(vehicle3.getPhysicalVehicle());
        assertTrue(chargingStation2.isOccupied());
    }

    @Test
    public void setConsumption() {
        // Create ChargingStation
        ChargingStation chargingStation = new ChargingStation();

        // Test consumption 0
        chargingStation.setConsumption(chargingStation.getConsumption() + 0);
        Assert.assertTrue(chargingStation.getConsumption() == 0);

        // Test consumption 100
        chargingStation.setConsumption(chargingStation.getConsumption() + 100);
        Assert.assertTrue(chargingStation.getConsumption() == 100);
    }
}
