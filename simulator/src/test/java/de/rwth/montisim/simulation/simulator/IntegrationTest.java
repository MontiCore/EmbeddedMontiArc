package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.task.Task;
import de.rwth.montisim.simulation.vehicle.task.goal.MetricGoal;
import de.rwth.montisim.simulation.vehicle.task.goal.PathGoal;
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
    final private String MAP_PATH = "src/test/resources/aachen.osm";
    private OsmMap osmMap;
    private World world;

    @Before
    public void initResource() throws Exception {
        osmMap = new OsmMap("myMap", new File(MAP_PATH));
        world = new OsmToWorldLoader(osmMap).getWorld();
        world.finalizeGraph();
    }

    @Test
    public void driveToTargetLocalCoordinate() throws Exception {
        Vec3 startPos = new Vec3(1, 0, 0);
        Vec2 targetPos = new Vec2(-63.83, -171.96);

        Task task = new Task();
        task.addGoal(PathGoal.newBuilder()
                .eventually()
                .arrive(startPos.asVec2())
                .arrive(targetPos)
                .withInRange(10)
                .build());
        task.addGoal(MetricGoal.newBuilder()
                .setProperty("speed")
                .never()
                .greater(1000, "m/s")
                .build());

        driveToTarget(task);
    }

    @Test
    public void driveToTargetGlobalCoordinate() throws Exception {
        Coordinates startCoord = new Coordinates(6.0721450,50.7738916);
        Coordinates targetCoord = new Coordinates(6.0690220,50.773491);

        Task task = new Task();
        task.addGoal(PathGoal.newBuilder()
                .eventually()
                .arrive(startCoord)
                .arrive(targetCoord)
                .withInRange(10)
                .build());

        task.getPathGoals().forEach(g -> g.convertCoordinate(osmMap, world));
        driveToTarget(task);
    }

    @Test
    public void driveToTargetGlobalOsmId() throws Exception {
        long startOsmId = 3369847019l;
        long targetOsmId = 444540939l;

        Task task = new Task();
        task.addGoal(PathGoal.newBuilder()
                .eventually()
                .arrive(startOsmId)
                .arrive(targetOsmId)
                .withInRange(10)
                .build());

        task.getPathGoals().forEach(g -> g.convertCoordinate(osmMap, world));
        driveToTarget(task);
    }

    public void driveToTarget(Task task) throws Exception {
        Pathfinding pathfinding = new PathfindingImpl(world);
        MessageTypeManager mtManager = new MessageTypeManager();

        SimulationConfig config = new SimulationConfig();
        config.max_duration = Duration.ofSeconds(1000);
        Simulator simulator = new Simulator(config, world, pathfinding, mtManager);
        SimulationLoop loop = new SimulationLoop(simulator, config);

        DefaultVehicleConfig vConf = DefaultVehicleConfig.withJavaAutopilot().setTask(task);
        Vehicle vehicle = simulator.getVehicleBuilder(vConf.properties).setName("TestVehicle").build();

        // Read positions from task
        PathGoal pg = task.getPathGoals().get(0);
        Vec2 startPos = pg.getPath().get(0);
        Vec2 targetPos = pg.getPath().get(pg.getPath().size() - 1);

        vehicle.physicsModel.setGroundPosition(new Vec3(0, 0, 0), new Vec2(startPos.x, startPos.y));

        simulator.addSimulationObject(vehicle);

        Navigation nav = (Navigation) vehicle.eesystem.getComponentManager().getComponent("Navigation").get();
        nav.pushTargetPos(targetPos);

        vehicle.properties.setCoordinates(new Coordinates());
        world.converter.get().metersToCoords(startPos, vehicle.properties.getCoordinates());

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

            double dist = vehicle.physicalObject.pos.distance(new Vec3(targetPos.x, targetPos.y, 0));
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

        double dist = vehicle.physicalObject.pos.distance(new Vec3(targetPos.x, targetPos.y, 0));
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