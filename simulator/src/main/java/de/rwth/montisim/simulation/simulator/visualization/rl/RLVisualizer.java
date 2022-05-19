package de.rwth.montisim.simulation.simulator.visualization.rl;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.RLSimulationHandler;
import de.rwth.montisim.simulation.simulator.SimulationConfig;
import de.rwth.montisim.simulation.simulator.Simulator;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.PathfinderRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.WorldRenderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.Control;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import javax.swing.*;
import java.awt.Dimension;
import java.time.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

// Reinforcement learning version of ScenarioVis

public class RLVisualizer {

    String current_scenario = "";
    JLabel scenario_name;

    Control control;
    Viewer2D viewer;
    private List<CarRenderer> carRenderers = new ArrayList<>();

    Simulator simulator;
    SimulationConfig simConfig;
    OsmMap map;
    public Instant simTime;
    RLSimulationHandler rlSimulationHandler;
    private boolean done_clearing = true;


    public RLVisualizer(OsmMap map, SimulationConfig simConfig, Viewer2D viewer, Instant simTime) {
        this.map = map;
        this.simConfig = simConfig;
        this.viewer = viewer;
        this.simTime = simTime;
    }

    // initialize simulation handler
    public void init(Boolean distributed, Boolean randomize, Boolean play, Boolean miniStep, String selfPlay_mode) {
        String lib_path = System.getProperty("user.dir") + "/";
        System.load(lib_path + "libROSInterface.so");
        rlSimulationHandler = new RLSimulationHandler(simConfig, simTime, map, this);
        rlSimulationHandler.setSettings(distributed, randomize, play, miniStep, selfPlay_mode);
        new Thread(() -> rlSimulationHandler.start()).start();
    }

    public void clearRenderer() {
        done_clearing = false;
        if (viewer != null && !carRenderers.isEmpty()) {
            viewer.clearRenderers();
            carRenderers.clear();
        }
        done_clearing = true;

    }

    public void setup(World world, Pathfinding pathfinding) {
        // Setup visualizer
        viewer.addRenderer(new WorldRenderer(world));
        viewer.addRenderer(new PathfinderRenderer(pathfinding));

        // Init CarRenderers and find view for all Vehicles
        Collection<Vehicle> vehicles = rlSimulationHandler.getSim().getVehicles().collect(Collectors.toList());
        setView(vehicles);
        for (Vehicle v : vehicles) {
            CarRenderer cr = new CarRenderer();
            cr.setCar(v);
            viewer.addRenderer(cr);
            carRenderers.add(cr);
        }
        while (!done_clearing) ;
        viewer.repaint();

    }

    void setView(Collection<Vehicle> vehicles) {
        Vec2 avg_pos = new Vec2(0, 0);
        int count = 0;
        Vec2 min_pos = new Vec2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        Vec2 max_pos = new Vec2(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
        Vec2 vpos = new Vec2();
        for (Vehicle v : vehicles) {
            vpos.set(v.physicalObject.pos);
            IPM.add(avg_pos, vpos);
            if (vpos.x < min_pos.x)
                min_pos.x = vpos.x;
            if (vpos.y < min_pos.y)
                min_pos.y = vpos.y;
            if (vpos.x > max_pos.x)
                max_pos.x = vpos.x;
            if (vpos.y > max_pos.y)
                max_pos.y = vpos.y;
            ++count;
        }
        if (count == 0) {
            viewer.setCenter(avg_pos);
            viewer.setZoom(4);
            return;
        }

        IPM.multiply(avg_pos, 1.0 / (double) count);
        viewer.setCenter(avg_pos);

        Vec2 range = new Vec2();
        IPM.subtractTo(range, max_pos, min_pos);
        IPM.add(range, new Vec2(16, 16)); // Margin
        Dimension d = viewer.getSize();
        double xscale = d.getWidth() / range.x;
        double yscale = d.getHeight() / range.y;
        double scale = 20;
        if (xscale < scale)
            scale = xscale;
        if (yscale < scale)
            scale = yscale;
        viewer.setZoom(scale);
    }

    public void redraw() {
        for (CarRenderer cr : carRenderers)
            cr.dirty = true;
        while (!done_clearing) ;
        viewer.update();
    }
}