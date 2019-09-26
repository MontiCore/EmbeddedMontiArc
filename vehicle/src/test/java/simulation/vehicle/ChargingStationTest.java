/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.junit.*;
import static org.junit.Assert.*;
import simulation.environment.util.ChargingProcess;
import simulation.environment.object.ChargingStation;
import simulation.environment.util.VehicleType;


/**
 * Charging Station Test Class
 *
 * @version 1.0
 * @since 2019-09-17
 */
public class ChargingStationTest {

    @Test
    public void startCharging() {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation = new ChargingStation();

        // Test for non-electric Vehicle
        PhysicalVehicle vehicleNotElectric = new ModelicaPhysicalVehicle();
        assertFalse(chargingStation.startCharging(vehicleNotElectric));

        // Test for car that is not standing near CS
        PhysicalVehicle vehicleNotNear = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,25);
        vehicleNotNear.initPhysics();
        vehicleNotNear.setPosition(new ArrayRealVector(new double[]{10,5,0}));
        assertFalse(chargingStation.startCharging(vehicleNotNear));

        // Test for electric car standing near CS
        PhysicalVehicle vehicle = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,10);
        assertTrue(chargingStation.startCharging(vehicle));
        assertTrue(chargingStation.getCarObjects().size()==1);

        // Test for occupied CS
        PhysicalVehicle vehicle2 = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,10);
        assertFalse(chargingStation.startCharging(vehicle2));
    }

    @Test
    public void stopCharging() {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation = new ChargingStation();

        // Create and charge vehicle
        PhysicalVehicle vehicle = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,50);
        chargingStation.startCharging(vehicle);

        // Test for vehicle that is not charged at the CS
        PhysicalVehicle vehicleNotCharged = new MassPointPhysicalVehicle(VehicleType.ELECTRIC,80);
        assertFalse(chargingStation.stopCharging(vehicleNotCharged));

        // Stop charging
        assertTrue(chargingStation.stopCharging(vehicle));
        assertTrue(chargingStation.getCarObjects().isEmpty());
        assertFalse(chargingStation.stopCharging(vehicle));
    }

    @Test
    public void isOccupied() {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation1 = new ChargingStation();
        // Create ChargingStation with capacity=3
        ChargingStation chargingStation2 = new ChargingStation();
        chargingStation2.setCapacity(3);

        // Test ChargingStation not occupied
        assertFalse(chargingStation1.isOccupied());
        assertFalse(chargingStation2.isOccupied());

        // Create Vehicle
        ModelicaPhysicalVehicle vehicle1 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 10.0);
        ModelicaPhysicalVehicle vehicle2 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 20.0);
        ModelicaPhysicalVehicle vehicle3 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 30.0);

        // Test ChargingStation occupied for ChargingStation1
        chargingStation1.startCharging(vehicle1);
        assertTrue(chargingStation1.isOccupied());

        // Test ChargingStation occupied for ChargingStation2
        chargingStation2.startCharging(vehicle1);
        assertFalse(chargingStation2.isOccupied());
        chargingStation2.startCharging(vehicle2);
        assertFalse(chargingStation2.isOccupied());
        chargingStation2.startCharging(vehicle3);
        assertTrue(chargingStation2.isOccupied());
    }

    @Test
    public void setConsumption() {
        // Create ChargingStation
        ChargingStation chargingStation = new ChargingStation();

        // Test consumption 0
        chargingStation.setConsumption(chargingStation.getConsumption()+0);
        Assert.assertTrue(chargingStation.getConsumption() == 0);

        // Test consumption 100
        chargingStation.setConsumption(chargingStation.getConsumption()+100);
        Assert.assertTrue(chargingStation.getConsumption() == 100);
    }
}
