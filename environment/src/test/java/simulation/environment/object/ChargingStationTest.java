package simulation.environment.object;

import org.junit.*;
import static org.junit.Assert.*;
import simulation.environment.util.ChargingProcess;
import simulation.environment.object.ChargingStation;


/**
 * Created by ChargingStation Team on 02.09.2019
 */
public class ChargingStationTest {

    @Test
    public void isOccupied() {
        // Create ChargingStation
        ChargingStation chargingStation = new ChargingStation();
        // Test ChargingStation not occupied
        Assert.assertFalse(chargingStation.isOccupied());

        // Create Vehicle
        ModelicaPhysicalVehicle vehicle = ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 100);
        // Test ChargingStation occupied
        chargingStation.startCharging(vehicle);
        Assert.assertTrue(chargingStation.isOccupied());
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
