/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.*;
import java.awt.BorderLayout;
import java.time.*;
import java.util.logging.Logger;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.simulator.vehicleconfigs.TestVehicleConfig;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.plotter.TimePlotter;
import de.rwth.montisim.simulation.simulator.visualization.ui.*;
import de.rwth.montisim.simulation.vehicle.*;
import de.rwth.montisim.simulation.vehicle.VehicleProperties.BuildContext;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;

public class PhysicsDebug extends JFrame implements SimulationRunner {
    public static void main(String args[]) throws EESetupException {
        new PhysicsDebug(args);
    }

    private static final Vec3 START_DIR = new Vec3(1, 0, 0);
    private static final Vec3 START_CROSS = new Vec3(0, 1, 0);

    private static final long serialVersionUID = 1L;

    final int PHYSICS_TICK_DURATION_MS = 10;
    final int TARGET_FPS = 30;
    final int MIN_FPS = 2;

    final Viewer2D viewer;
    final Control control;
    final CarRenderer cr;
    final TestVehicleConfig setup;
    final TimePlotter plotter;

    MessageTypeManager mtManager;
    Vehicle vehicle;
    RigidbodyPhysics physics;

    boolean postedMsg = false;
    boolean crossedQuarter = false;

    public PhysicsDebug(String args[]) throws EESetupException {
        super("Physics Debug");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(900, 900);

        UIInfo.antialiasing = true;

        viewer = new Viewer2D();

        // vis1(args);

        setup = new TestVehicleConfig();
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

    private void setup() throws EESetupException {
        postedMsg = false;
        crossedQuarter = false;

        mtManager = new MessageTypeManager();
        VehicleProperties config = setupTurningCar();
        try {
            vehicle = VehicleBuilder.fromConfig(new BuildContext(null, mtManager), config).setName("TestVehicle").build();
            physics = (RigidbodyPhysics) vehicle.physicsModel;
            physics.setGroundPosition(new Vec3(0, 0, 0), new Vec2(START_DIR.x, START_DIR.y));
            VehicleProperties p = vehicle.properties;
            cr.setCar(vehicle, new Vec3(p.body.length, p.body.width, p.body.height));
        } catch (SerializationException | EEMessageTypeException e) {
            e.printStackTrace();
        }
    }

    private VehicleProperties setupTurningCar(){
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

    @Override
    public void update(TimeUpdate newTime) {
        vehicle.update(newTime);

        // Measure Flipping Speed
        if (!postedMsg && Math.acos(vehicle.physicalObject.rotation.col3.z)*Geometry.RAD_TO_DEG >= 5){
            Logger.getGlobal().info("Speed at 5 deg tilt (km/h): "+(vehicle.physicalObject.velocity.magnitude()*3.6));
            postedMsg = true;
        }
    }
}