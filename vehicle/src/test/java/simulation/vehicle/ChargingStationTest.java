package simulation.vehicle;

import org.junit.*;
import static org.junit.Assert.*;
import simulation.environment.util.ChargingProcess;
import simulation.environment.object.ChargingStation;
// TODO
// import simulation.vehicle.ModelicaPhysicalVehicle;
// import simulation.environment.util.VehicleType;


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
        // Create ChargingStation
        ChargingStation chargingStation = new ChargingStation();
        // Test ChargingStation not occupied
        Assert.assertFalse(chargingStation.isOccupied());


        // Create Vehicle

        // TODO

        /*
        ModelicaPhysicalVehicle vehicle = ModelicaPhysicalVehicle(VehicleType.ELECTRIC, 100.0);
        // Test ChargingStation occupied
        chargingStation.startCharging(vehicle);
        Assert.assertTrue(chargingStation.isOccupied());
         */
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
