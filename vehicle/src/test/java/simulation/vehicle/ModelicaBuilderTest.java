package simulation.vehicle;

import org.junit.*;

/**
 * Class that tests the ModelicaPhysicalVehicleBuilder class
 */
public class ModelicaBuilderTest {

    @Test
    public void justATest(){
        PhysicalVehicle physicalVehicle1 = new ModelicaPhysicalVehicleBuilder().buildPhysicalVehicle();
        Assert.assertTrue(physicalVehicle1.getPhysicalVehicleInitialized());
    }
}
