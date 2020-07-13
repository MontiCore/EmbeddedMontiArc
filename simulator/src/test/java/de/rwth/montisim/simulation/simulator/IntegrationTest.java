package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.time.Duration;

import org.junit.*;

import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eecomponents.navigation.Navigation;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.vehicleconfigs.DefaultVehicleConfig;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public class IntegrationTest {
    @Test
    public void driveToTarget() throws Exception {
        final Vec3 START_DIR = new Vec3(1, 0, 0);
        final Vec2 TARGET_POS = new Vec2(-63.83, -171.96);

        //System.out.println("Working Dir: " + LibraryService.getWorkingDirectory());
        World world = new OsmToWorldLoader(new OsmMap("aachen", new File("src/test/resources/aachen.osm"))).getWorld();
        Pathfinding pathfinding = new PathfindingImpl(world);
        MessageTypeManager mtManager = new MessageTypeManager();

        SimulationConfig config = new SimulationConfig();
        config.max_duration = Duration.ofSeconds(300);
        Simulator simulator = new Simulator(config, world, pathfinding, mtManager);
        SimulationLoop loop = new SimulationLoop(simulator, config);



        DefaultVehicleConfig vConf = DefaultVehicleConfig.withJavaAutopilot();
        
        Vehicle vehicle = simulator.getVehicleBuilder(vConf.properties).setName("TestVehicle").build();
        vehicle.physicsModel.setGroundPosition(new Vec3(0, 0, 0), new Vec2(START_DIR.x, START_DIR.y));
        simulator.addSimulationObject(vehicle);
        
        
        Navigation nav = (Navigation) vehicle.eesimulator.getComponentManager().getComponent("Navigation").get();
        nav.pushTargetPos(TARGET_POS);

        vehicle.addTarget(TARGET_POS);

        Assert.assertTrue("Simulation Error", loop.run());
        Assert.assertTrue("Vehicle did not reach target", simulator.allTasksSucceeded());
        // TODO start at (0,0)
        // Go to (-63.83, -171.96) in less than 45 secs
        // TODO start at better position & orientation
        // TODO add temporary "objective checker"
    }
}