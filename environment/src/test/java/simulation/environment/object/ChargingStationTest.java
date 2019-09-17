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
    public void testApp() throws Exception {
        // Create ChargingStation
        ChargingStation chargingStation = new ChargingStation();
        // Create Vehicle
        ModelicaPhysicalVehicleBuilder vehicleBuilder = new ModelicaPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = vehicleBuilder.buildPhysicalVehicle();

    }

    // ================================
    // TODO Test the following Methodes
    // ================================
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
        // TODO
    }

    @Test
    public void carStandingAtTheCS() {
        // TODO
    }
}
