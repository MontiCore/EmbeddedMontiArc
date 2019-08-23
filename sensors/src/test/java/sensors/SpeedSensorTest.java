/* (c) https://github.com/MontiCore/monticore */
package sensors;

import static org.junit.Assert.assertTrue;
import org.junit.Test;
import commons.simulation.Sensor;
import simulation.vehicle.PhysicalVehicle;
import simulation.vehicle.MassPointPhysicalVehicleBuilder;

/**
 * Created by Aklima Zaman on 19-Dec-16.
 */
public class SpeedSensorTest {

    @Test
    public void SpeedSensorTest() {
        MassPointPhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = physicalVehicleBuilder.buildPhysicalVehicle();
        Sensor speedSensor = new SpeedSensor(physicalVehicle);

        assertTrue(!physicalVehicle.getVelocity().equals(speedSensor.getValue()));

    }
}
