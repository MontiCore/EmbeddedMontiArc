package de.rwth.montisim.simulation.simulator.visualization;


import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSeparator;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.plotter.TimePlotter;
import de.rwth.montisim.simulation.simulator.visualization.ui.Control;
import de.rwth.montisim.simulation.simulator.visualization.ui.SimulationRunner;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.vehiclesetups.DefaultVehicleSetup;

import java.awt.BorderLayout;
import java.time.Duration;
import java.time.Instant;

public class PhysicsDebug extends JFrame implements SimulationRunner {
    public static void main(String args[]) throws EESetupException {
        new PhysicsDebug(args);
    }


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
    final DefaultVehicleSetup setup;
    final TimePlotter plotter;

    Vehicle vehicle;
    RigidbodyPhysics physics;


    public PhysicsDebug(String args[]) throws EESetupException {
        super("Physics Debug");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(900, 900);

        UIInfo.antialiasing = true;

        viewer = new Viewer2D();

        // vis1(args);


        //timer = new Timer((int)(FRAME_DURATION*1000), this);

        setup = new DefaultVehicleSetup();
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
        // add(new JLabel("Map Visualization."), BorderLayout.NORTH);
        add(topPanel, BorderLayout.PAGE_START);
        add(viewer, BorderLayout.CENTER);
        //add(plotter, BorderLayout.PAGE_END);
        setVisible(true);
    }

    private void setup() throws EESetupException {
        simTime = Instant.EPOCH;
        vehicle = new Vehicle(setup);
        physics = (RigidbodyPhysics) vehicle.physics_model;
        physics.setGroundPosition(new Vec3(0, 0, 0), new Vec2(1, 0));
        // IPM.rotationMatrixToMat(0.2, car.rotation);
        physics.gasValue.set(1);
        physics.gas.targetValue = 1;
        //physics.steeringValue.set(30);

        VehicleProperties p = vehicle.properties;

        cr.setCar(physics.getPhysicalObject(), new Vec3(p.body.length, p.body.width, p.body.height));
    }

    @Override
    public long run(Instant timePoint) {
        vehicle.update(new TimeUpdate(simTime, dt));
        simTime = simTime.plus(dt);
        //physics.update(tu);
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