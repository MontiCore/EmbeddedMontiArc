package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.JFrame;
import javax.swing.Timer;

import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;
import de.rwth.montisim.simulation.vehicle.DefaultVehicleSetup;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.time.Duration;
import java.time.Instant;

public class PhysicsDebug extends JFrame implements ActionListener {
    private static final long serialVersionUID = 1L;

    final long PHYSICS_TICK_DURATION_MS = 10;
    final long TARGET_FPS = 30;

    final double TICK_DURATION = PHYSICS_TICK_DURATION_MS / 1000.0;
    final double FRAME_DURATION = 1.0 / TARGET_FPS;
    final long TICK_NANO = PHYSICS_TICK_DURATION_MS * 1000000;
    TimeUpdate tu = new TimeUpdate(Instant.EPOCH, Duration.ofMillis(PHYSICS_TICK_DURATION_MS));

    Viewer2D viewer;
    CarRenderer cr;
    Timer timer;

    Vehicle vehicle;
    RigidbodyPhysics physics;

    long last_tick;

    public PhysicsDebug(String args[]) throws EESetupException {
        super("Physics Debug");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(900, 900);

        UIInfo.antialiasing = true;

        viewer = new Viewer2D();

        // vis1(args);


        timer = new Timer((int)(FRAME_DURATION*1000), this);

        DefaultVehicleSetup setup = new DefaultVehicleSetup();
        vehicle = new Vehicle(setup);
        physics = (RigidbodyPhysics) vehicle.physics_model;
        physics.setGroundPosition(new Vec3(0, 0, 0), new Vec2(1, 0));
        // IPM.rotationMatrixToMat(0.2, car.rotation);

        cr = new CarRenderer(physics.getPhysicalObject(), new Vec3(4, 2, 1));
        viewer.addRenderer(cr);
        viewer.scale = 20;

        physics.gasValue.set(1);
        physics.steeringValue.set(30);

        setLayout(new BorderLayout());
        // add(new JLabel("Map Visualization."), BorderLayout.NORTH);
        add(viewer, BorderLayout.CENTER);
        setVisible(true);
        timer.start();
        last_tick = System.nanoTime();
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if (e.getSource() == timer) {
            // Simulate
            long new_tick = System.nanoTime();
            while (last_tick < new_tick) {
                physics.update(tu);
                last_tick += TICK_NANO;
            }
            // Re-draw
            cr.dirty = true;
            viewer.update();
        }
    }
}