package de.rwth.montisim.simulation.simulator;

import java.io.File;
import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Comparator;
import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;
import de.rwth.montisim.simulation.vehicle.task.metric.MetricGoalProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

import org.junit.*;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
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
        Vec2 startPos = new Vec2(1, 0);
        Vec2 targetPos = new Vec2(-63.83, -171.96);

        TaskProperties task = new TaskProperties();
        task.addGoal(new PathGoalProperties()
                .reach(targetPos)
                .withinRange(10)
                .eventually());
        task.addGoal(new MetricGoalProperties()
                .compare("true_velocity")
                .with(1000, "m/s")
                .operator(Comparator.GREATER)
                .never());

        driveToTarget(startPos, targetPos, task);
    }

    @Test
    public void driveToTargetGlobalCoordinate() throws Exception {
        Coordinates startCoord = new Coordinates(6.0721450,50.7738916);
        Coordinates targetCoord = new Coordinates(6.0690220,50.773491);

        TaskProperties task = new TaskProperties();
        task.addGoal(new PathGoalProperties()
                .reach(targetCoord)
                .withinRange(10)
                .eventually());

        Vec2 startPos = new Vec2();
        Vec2 targetPos = new Vec2();
        world.converter.get().coordsToMeters(startCoord, startPos);
        world.converter.get().coordsToMeters(targetCoord, targetPos);
        driveToTarget(startPos, targetPos, task);
    }

    @Test
    public void driveToTargetGlobalOsmId() throws Exception {
        long startOsmId = 3369847019l;
        long targetOsmId = 444540939l;

        TaskProperties task = new TaskProperties();
        task.addGoal(new PathGoalProperties()
                .reach(targetOsmId)
                .withinRange(10)
                .eventually());

        Vec2 startPos = new Vec2();
        Vec2 targetPos = new Vec2();
        world.converter.get().coordsToMeters(osmMap.getNode(startOsmId).coords, startPos);
        world.converter.get().coordsToMeters(osmMap.getNode(targetOsmId).coords, targetPos);
        driveToTarget(startPos, targetPos, task);
    }

    public void driveToTarget(Vec2 startPos, Vec2 targetPos, TaskProperties task) throws Exception {
        Pathfinding pathfinding = new PathfindingImpl(world);
        MessageTypeManager mtManager = new MessageTypeManager();

        SimulationConfig config = new SimulationConfig();
        config.max_duration = Duration.ofSeconds(1000);

        DefaultVehicleConfig vConf = DefaultVehicleConfig.withJavaAutopilot().setTask(task);
        vConf.properties.start_pos = startPos;
        config.cars.add(vConf.properties.setName("TestVehicle"));

        Simulator simulator = config.build(world, pathfinding, mtManager, osmMap);

        Vehicle vehicle = simulator.getVehicle("TestVehicle");

        Assert.assertFalse(simulator.allTasksSucceeded());

        // dump and reload vehicle every 1000 steps. test if it is able to reach the destination.
        Instant simulationTime = config.start_time;
        int cnt = 0;
        while (true) {
            TaskStatus status = simulator.status();
            if (status == TaskStatus.SUCCEEDED) break;
            if (status == TaskStatus.FAILED) throw new IllegalStateException("Vehicle did not reach target");

            TimeUpdate tu = new TimeUpdate(simulationTime, config.tick_duration);
            simulator.update(tu);
            simulationTime = tu.newTime;

            if (++cnt % 1000 != 0) {
                continue;
            }

            double dist = vehicle.physicalObject.pos.distance(new Vec3(targetPos.x, targetPos.y, 0));
            System.out.printf("dist: %.2fm, pos: %s\n", dist, vehicle.physicalObject.pos);

            System.out.println("Dumping and reloading vehicle...");
            String state = vehicle.stateToJson();
            simulator.removeSimulationObject(vehicle);

            vehicle = VehicleBuilder.fromJsonState(simulator.buildContext, state).build();
            simulator.addSimulationObject(vehicle);
        }

        double dist = vehicle.physicalObject.pos.distance(new Vec3(targetPos.x, targetPos.y, 0));
        System.out.printf("dist: %.2fm, pos: %s\n", dist, vehicle.physicalObject.pos);

    }
}