/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.*;
import java.awt.BorderLayout;
import java.time.*;
import java.util.logging.Logger;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eecomponents.autopilots.TestAutopilotProperties;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.plotter.TimePlotter;
import de.rwth.montisim.simulation.simulator.visualization.ui.*;
import de.rwth.montisim.simulation.vehicle.*;
import de.rwth.montisim.simulation.vehicle.config.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;

public class PhysicsDebug extends JFrame implements SimulationRunner {
    public static void main(String args[]) throws EESetupException {
        new PhysicsDebug(args);
    }

    private static final Vec3 START_DIR = new Vec3(1, 0, 0);
    private static final Vec3 START_CROSS = new Vec3(0, 1, 0);

    private static final long serialVersionUID = 1L;

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

    private void setup() throws EESetupException {
        simTime = Instant.EPOCH;
        postedMsg = false;
        crossedQuarter = false;

        mtManager = new MessageTypeManager();
        VehicleConfig config = setupTurningCar();
        try {
            vehicle = new VehicleBuilder(mtManager, null, config).setName("TestVehicle").build();
            physics = (RigidbodyPhysics) vehicle.physicsModel;
            physics.setGroundPosition(new Vec3(0, 0, 0), new Vec2(START_DIR.x, START_DIR.y));
            VehicleProperties p = vehicle.properties;
            cr.setCar(physics.getPhysicalObject(), new Vec3(p.body.length, p.body.width, p.body.height));
        } catch (EEMessageTypeException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    private VehicleConfig setupTurningCar(){
        TestVehicleConfig config = new TestVehicleConfig();

        double maxForce = config.electricalPTProperties.motorProperties.motorPeekTorque *
            config.electricalPTProperties.transmissionRatio * 2 /
            config.properties.wheels.wheelDiameter;
        
        double turnRadius = 30;
        double maxSpeed = 0.8*270*Math.pow(turnRadius, -0.5614);
        System.out.println("MaxSpeed: "+Double.toString(maxSpeed));

        config.eeConfig.addComponent(
            TestAutopilotProperties.circleAutopilot(Duration.ofMillis(1), maxForce/config.properties.body.mass, maxSpeed, turnRadius).setName("TestAutopilot")
        );

        return config;
    }

    @Override
    public long run(Instant timePoint) {
        vehicle.update(new TimeUpdate(simTime, dt));

        // Measure turning radius
        // if (vehicle.physical_object.rotation.col1.dotProduct(START_DIR) <= 0){
        //     crossedQuarter = true;
        // }
        // if (crossedQuarter && !postedMsg && vehicle.physical_object.rotation.col1.dotProduct(START_CROSS) <= 0){
        //     Logger.getGlobal().info("Position at 180 deg turn: "+vehicle.physical_object.pos);
        //     postedMsg = true;
        // }

        // Measure Flipping Speed
        if (!postedMsg && Math.acos(vehicle.physicalObject.rotation.col3.z)*Geometry.RAD_TO_DEG >= 5){
            Logger.getGlobal().info("Speed at 5 deg tilt (km/h): "+(vehicle.physicalObject.velocity.magnitude()*3.6));
            postedMsg = true;
        }

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