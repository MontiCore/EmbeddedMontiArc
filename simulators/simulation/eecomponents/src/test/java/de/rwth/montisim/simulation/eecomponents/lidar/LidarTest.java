package de.rwth.montisim.simulation.eecomponents.lidar;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Building;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import org.junit.Assert;
import org.junit.Test;

import static org.mockito.Mockito.mock;

public class LidarTest {

    @Test
    public void computeShortestDistanceTest() {
        World world = new World("Stub");
        Building building = new Building("Testbuilding", 0, "Testtype", 20d, 3);
        building.boundary.add(new Vec3(-30, 80, 0));
        building.boundary.add(new Vec3(-30, 50, 0));
        building.boundary.add(new Vec3(10, 50, 0));
        building.boundary.add(new Vec3(10, 80, 0));
        world.buildings.add(building);

        Vec2 vehiclePos = new Vec2(0, 0);
        Vec2 ray = new Vec2(0, 1);

        LidarProperties lidarProperties = mock(LidarProperties.class);
        EESystem eeSystem = mock(EESystem.class);
        Vehicle vehicle = mock(Vehicle.class);
        Lidar lidar = new Lidar(lidarProperties, eeSystem, world, vehicle);


        Assert.assertEquals(50d, lidar.computeShortestDistance(vehiclePos, ray), Lidar.DOUBLE_TOLERANCE);

    }

    @Test
    public void computeShortestDistanceInfinityTest() {
        World world = new World("Stub");

        Vec2 vehiclePos = new Vec2(0, 0);
        Vec2 ray = new Vec2(0, 1);

        LidarProperties lidarProperties = mock(LidarProperties.class);
        EESystem eeSystem = mock(EESystem.class);
        Vehicle vehicle = mock(Vehicle.class);
        Lidar lidar = new Lidar(lidarProperties, eeSystem, world, vehicle);


        Assert.assertEquals(Double.MAX_VALUE, lidar.computeShortestDistance(vehiclePos, ray), Lidar.DOUBLE_TOLERANCE);
    }
}
