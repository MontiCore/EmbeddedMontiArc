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
        Assert.assertFalse(chargingStation.isOccupied());

        /*
        try {
            Assert.assertFalse(chargingStation.isOccupied());
        } catch (AssertionError e) {
            System.out.println("New ChargingStation is Occupied");
            throw e;
        }

        // Create Vehicle
        ModelicaPhysicalVehicleBuilder vehicleBuilder = new ModelicaPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = vehicleBuilder.buildPhysicalVehicle();
        try {
            Assert.assertTrue(chargingStation.isOccupied());
        } catch (AssertionError e) {
            System.out.println("New ChargingStation is Occupied");
            throw e;
        }
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
