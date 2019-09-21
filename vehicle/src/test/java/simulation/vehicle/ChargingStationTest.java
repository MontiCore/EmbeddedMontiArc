/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle;

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
        // TODO
    }

    @Test
    public void stopCharging() {
        // TODO
    }

    @Test
    public void isOccupied() {
        // Create ChargingStation with capacity=1
        ChargingStation chargingStation1 = new ChargingStation();
        // Create ChargingStation with capacity=3
        ChargingStation chargingStation2 = new ChargingStation();
        chargingStation2.setCapacity(3);

        // Test ChargingStation not occupied
        Assert.assertFalse(chargingStation1.isOccupied());
        Assert.assertFalse(chargingStation2.isOccupied());

        // Create Vehicle
        ModelicaPhysicalVehicle vehicle1 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 10.0);
        ModelicaPhysicalVehicle vehicle2 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 20.0);
        ModelicaPhysicalVehicle vehicle3 = new ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 30.0);

        // Test ChargingStation occupied for ChargingStation1
        chargingStation1.startCharging(vehicle1);
        Assert.assertTrue(chargingStation1.isOccupied());

        // Test ChargingStation occupied for ChargingStation2
        chargingStation2.startCharging(vehicle1);
        Assert.assertFalse(chargingStation2.isOccupied());
        chargingStation2.startCharging(vehicle2);
        Assert.assertFalse(chargingStation2.isOccupied());
        chargingStation2.startCharging(vehicle3);
        Assert.assertTrue(chargingStation2.isOccupied());
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
