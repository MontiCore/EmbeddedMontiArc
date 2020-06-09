/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.*;
import java.awt.BorderLayout;
import java.io.File;
import java.time.*;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.eecomponents.navigation.Navigation;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.pathfinding.*;
import de.rwth.montisim.simulation.simulator.*;
import de.rwth.montisim.simulation.simulator.vehicleconfigs.*;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.*;
import de.rwth.montisim.simulation.simulator.visualization.plotter.TimePlotter;
import de.rwth.montisim.simulation.simulator.visualization.ui.*;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.config.VehicleConfig;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;


public class SimulationVisualizer extends JFrame implements SimulationRunner {
    private static final long serialVersionUID = -8677459653174721311L;

    public static final boolean SHOW_SEGMENTS = true;
    public static void main(String args[]) throws EESetupException {
        new SimulationVisualizer(args);
    }
    private static final Vec3 START_DIR = new Vec3(1, 0, 0);

    final long PHYSICS_TICK_DURATION_MS = 10;
    final long TARGET_FPS = 30;

    final double TICK_DURATION = PHYSICS_TICK_DURATION_MS / 1000.0;
    final double FRAME_DURATION = 1.0 / TARGET_FPS;
    final long TICK_NANO = PHYSICS_TICK_DURATION_MS * 1000000;
    Instant simTime = Instant.EPOCH;
    Duration dt = Duration.ofMillis(PHYSICS_TICK_DURATION_MS);

    final Viewer2D viewer;
    final Control control;
    final CarRenderer cr;
    final TimePlotter plotter;

    MessageTypeManager mtManager;
    Vehicle vehicle;
    RigidbodyPhysics physics;
    Simulator simulator;
    World world;
    Pathfinding pathfinding;

    boolean postedMsg = false;
    boolean crossedQuarter = false;

    public SimulationVisualizer(String args[]) throws EESetupException {
        super("SimulationVisualizer");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(900, 900);

        UIInfo.antialiasing = true;

        viewer = new Viewer2D();

        vis1(args);

        cr = new CarRenderer();
        viewer.addRenderer(cr);
        viewer.setZoom(20);

        setup();

        plotter = new TimePlotter();

        control = new Control(Control.Mode.SIMULATION, Instant.EPOCH, this);

        JPanel topPanel = new JPanel();
        topPanel.setLayout(new BoxLayout(topPanel, BoxLayout.Y_AXIS));
        topPanel.add(control);
        topPanel.add(new JSeparator());

        setLayout(new BorderLayout());
        add(topPanel, BorderLayout.PAGE_START);
        add(viewer, BorderLayout.CENTER);
        // add(plotter, BorderLayout.PAGE_END);
        setVisible(true);
    }

    public void vis1(String args[]) {
        String mapPath = "D:/EmbededMontiArc/basic-simulator/install/maps/aachen.osm";
        try {
            world = new OsmToWorldLoader(new OsmMap("aachen", new File(mapPath))).getWorld();
            viewer.addRenderer(new WorldRenderer(world, SHOW_SEGMENTS));
            pathfinding = new PathfindingImpl(world);
            viewer.addRenderer(new PathfinderRenderer(pathfinding));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void setup() throws EESetupException {
        simTime = Instant.EPOCH;
        postedMsg = false;
        crossedQuarter = false;

        mtManager = new MessageTypeManager();
        simulator = new Simulator(new SimulationConfig(), world, pathfinding, mtManager);

        //VehicleConfig config = setupTurningCar();
        VehicleConfig config = DefaultVehicleConfig.withJavaAutopilot();
        try {
            vehicle = simulator.getVehicleBuilder(config).setName("TestVehicle").build();
            simulator.addSimulationObject(vehicle);
            
            vehicle.physicsModel.setGroundPosition(new Vec3(0, 0, 0), new Vec2(START_DIR.x, START_DIR.y));

            cr.setCar(vehicle);

            Navigation nav = (Navigation) vehicle.eesimulator.getComponentManager().getComponent("Navigation").get();
            Vec2 TARGET_POS = new Vec2(-63.83, -171.96);
            //nav.pushTargetPos(new Vec2(384.77, -283.72));
            nav.pushTargetPos(TARGET_POS);
            vehicle.addTarget(TARGET_POS);
            
        } catch (EEMessageTypeException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private VehicleConfig setupTurningCar(){
        double turnRadius = 30;
        double maxSpeed = 0.8*270*Math.pow(turnRadius, -0.5614);
        System.out.println("MaxSpeed: "+Double.toString(maxSpeed));

        return TestVehicleConfig.newCircleAutopilotConfig(maxSpeed, turnRadius);
    }

    @Override
    public long run(Instant timePoint) {
        simulator.update(new TimeUpdate(simTime, dt));

        simTime = simTime.plus(dt);
        return TICK_NANO;
    }

    @Override
    public void redraw() {
        cr.dirty = true;
        viewer.update();
    }

    @Override
    public void reset() {
        try {
            setup();
        } catch (EESetupException e) {
            e.printStackTrace();
        }
    }
}