package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.Task;
import de.rwth.montisim.simulation.vehicle.task.goal.GoalBuilder;
import de.rwth.montisim.simulation.vehicle.task.goal.MetricGoal;
import de.rwth.montisim.simulation.vehicle.task.goal.PathGoal;
import de.rwth.montisim.simulation.vehicle.task.goal.VehicleProperty;
import org.junit.*;

import de.rwth.montisim.commons.map.Pathfinding;
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

        Task task = new Task();
        task.addGoal(PathGoal.newBuilder()
                .eventually()
                .arrive(TARGET_POS.x, TARGET_POS.y)
                .withInRange(10)
                .build());
        task.addGoal(MetricGoal.newBuilder()
                .setProperty(VehicleProperty.SPEED)
                .never()
                .greater(1000, "m/s")
                .build());
        vehicle.setTask(task);

        vehicle.physicsModel.setGroundPosition(new Vec3(0, 0, 0), new Vec2(START_DIR.x, START_DIR.y));

        simulator.addSimulationObject(vehicle);
        
        Navigation nav = (Navigation) vehicle.eesystem.getComponentManager().getComponent("Navigation").get();
        nav.pushTargetPos(TARGET_POS);

        Assert.assertFalse(simulator.allTasksSucceeded());

        // dump and reload vehicle every 1000 steps. test if it is able to reach the destination.
        Instant simulationTime = config.start_time;
        int cnt = 0;
        while (!simulator.finished()) {
//            cnt++;

            TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
            simulator.update(tu);
            simulationTime = tu.newTime;

            if (++cnt % 1000 != 0) {
                continue;
            }

            double dist = vehicle.physicalObject.pos.distance(new Vec3(TARGET_POS.x, TARGET_POS.y, 0));
            System.out.printf("dist: %.2fm, pos: %s\n", dist, vehicle.physicalObject.pos);

            System.out.println("Dumping and reloading vehicle...");
//            simulator = new Simulator(config, world, pathfinding, mtManager);
            simulator.getUpdatables().stream()
                    .filter(x -> x instanceof Vehicle)
                    .forEach(x -> simulator.removeSimulationObject((SimulationObject) x));

            String json = vehicle.stateToJson();
            VehicleProperties.BuildContext ctx = new VehicleProperties.BuildContext(pathfinding, mtManager);
            vehicle = VehicleBuilder.fromJsonState(ctx, json).build();
            simulator.addSimulationObject(vehicle);
        }

        double dist = vehicle.physicalObject.pos.distance(new Vec3(TARGET_POS.x, TARGET_POS.y, 0));
        System.out.printf("dist: %.2fm, pos: %s\n", dist, vehicle.physicalObject.pos);

//        vehicle.addTarget(TARGET_POS);

        Assert.assertTrue("Simulation Error", loop.run());
        Assert.assertTrue("Vehicle did not reach target", simulator.allTasksSucceeded());
        // TODO start at (0,0)
        // Go to (-63.83, -171.96) in less than 45 secs
        // TODO start at better position & orientation
        // TODO add temporary "objective checker"
    }
}