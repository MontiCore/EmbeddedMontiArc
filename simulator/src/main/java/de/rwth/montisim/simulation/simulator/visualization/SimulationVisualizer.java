/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.*;
import java.awt.BorderLayout;
import java.awt.event.*;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.*;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.SerializationException;
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
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;

public class SimulationVisualizer extends JFrame implements SimulationRunner {
    private static final long serialVersionUID = -8677459653174721311L;

    public static final boolean SHOW_SEGMENTS = true;

    // static class TestTimer implements ActionListener {
    //     Timer timer = new Timer(10, this);
    //     long calls = 0;
    //     long performed = 0;
    //     long expected_time;

    //     long lastTime;

    //     public TestTimer() {
    //         timer.start();
    //         lastTime = System.currentTimeMillis();
    //         expected_time = lastTime + 10;
    //     }

    //     @Override
    //     public void actionPerformed(ActionEvent e) {
    //         calls++;
    //         long max_time = expected_time + 10;
    //         expected_time += 10;
    //         long t = System.currentTimeMillis();
    //         if (t - lastTime > 1000){
    //             System.out.println("Calls in 1 sec: "+Long.toString(calls)+ ", performed: "+Long.toString(performed));
    //             lastTime = t;
    //             calls = 0;
    //             performed = 0;
    //         }

    //         if (t > max_time) return; // Timer calls catch up
            
    //         ++performed;
    //         // Delay
    //         int i = 0;
    //         do {
    //             i += i*i;
    //         } while((System.currentTimeMillis() - t) < 100);
    //     }
    // }

    public static void main(String args[]) throws EESetupException {
        //new TestTimer();
        new SimulationVisualizer(args);
    }

    private static final Vec3 START_DIR = new Vec3(1, 0, 0);

    final int PHYSICS_TICK_DURATION_MS = 10;
    final int TARGET_FPS = 30;
    final int MIN_FPS = 2;

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

        control = new Control(Control.Mode.SIMULATION, Instant.EPOCH, this, PHYSICS_TICK_DURATION_MS, TARGET_FPS, MIN_FPS);

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
        String mapPath = "simulator/src/test/resources/aachen.osm";
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
        postedMsg = false;
        crossedQuarter = false;

        mtManager = new MessageTypeManager();
        simulator = new Simulator(new SimulationConfig(), world, pathfinding, mtManager);

        // VehicleConfig config = setupTurningCar();
        VehicleProperties config = DefaultVehicleConfig.withJavaAutopilot().properties;


        try {
            vehicle = simulator.getVehicleBuilder(config).setName("TestVehicle").build();
            
            simulator.addSimulationObject(vehicle);

            vehicle.physicsModel.setGroundPosition(new Vec3(0, 0, 0), new Vec2(START_DIR.x, START_DIR.y));

            cr.setCar(vehicle);

            Navigation nav = (Navigation) vehicle.eesystem.getComponentManager().getComponent("Navigation").get();
            Vec2 TARGET_POS = new Vec2(-63.83, -171.96);
            // nav.pushTargetPos(new Vec2(384.77, -283.72));
            nav.pushTargetPos(TARGET_POS);
            vehicle.addTarget(TARGET_POS);

            printDebug();
        } catch (SerializationException | EEMessageTypeException e) {
            e.printStackTrace();
        }

    }

    private void printDebug() {
        try {
            Files.write(Paths.get("vehicle_config.json"), Json.toFormattedJson(vehicle.properties).getBytes());
            Files.write(Paths.get("vehicle_state.json"), vehicle.stateToJson().getBytes());
        } catch (SerializationException | IOException e1) {
            e1.printStackTrace();
        }
    }

    private VehicleProperties setupTurningCar() {
        double turnRadius = 30;
        double maxSpeed = 0.8*270*Math.pow(turnRadius, -0.5614);
        System.out.println("MaxSpeed: "+Double.toString(maxSpeed));

        return TestVehicleConfig.newCircleAutopilotConfig(maxSpeed, turnRadius).properties;
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

    
    int i = 0;
    int ci = 0;

    
    @Override
    public void update(TimeUpdate newTime) {
        simulator.update(newTime);
        if (i > 200){
            if (ci == 3){
                System.out.println("x");
            }
            try {
                String state = vehicle.stateToJson();
                Files.write(Paths.get("running_state.json"),state.getBytes());
                simulator.removeSimulationObject(vehicle);
                vehicle = VehicleBuilder.fromJsonState(simulator.buildContext, state).build();
                simulator.addSimulationObject(vehicle);
                cr.setCar(vehicle);
                System.out.println("Transfered Vehicle using serialization");
            } catch (SerializationException | IOException | EEMessageTypeException | EESetupException e1) {
                e1.printStackTrace();
            }
            i = 0;
            ci++;
        }
        i++;
    }
}