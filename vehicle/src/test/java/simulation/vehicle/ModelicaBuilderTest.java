package simulation.vehicle;

import org.junit.*;
import java.util.Optional;

/**
 * Class that tests the ModelicaPhysicalVehicleBuilder class
 */
public class ModelicaBuilderTest {

    @Test
    public void buildDefaultVehicle(){
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        PhysicalVehicle physicalVehicle = builder.buildPhysicalVehicle();

        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        Assert.assertEquals(Optional.empty(), vehicle.getControllerBus());
        Assert.assertEquals(Optional.empty(), vehicle.getController());
        Assert.assertEquals(Optional.empty(), vehicle.getNavigation());
        Assert.assertEquals(1750.0, vehicle.getMass(), 0);
    }

    @Test
    public void buildCustomVehicle(){
        ModelicaPhysicalVehicleBuilder builder = new ModelicaPhysicalVehicleBuilder();
        builder.setMass(1000.0);
        PhysicalVehicle physicalVehicle = builder.buildPhysicalVehicle();

        Vehicle vehicle = physicalVehicle.getSimulationVehicle();

        Assert.assertEquals(1000.0, vehicle.getMass(), 0);
    }
}
