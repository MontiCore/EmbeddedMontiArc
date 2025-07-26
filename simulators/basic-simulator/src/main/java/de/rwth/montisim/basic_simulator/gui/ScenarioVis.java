/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.IPM;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.SimulationConfig;
import de.rwth.montisim.simulation.simulator.Simulator;
import de.rwth.montisim.simulation.simulator.randomization.RandomRandomizationPropertiesPicker;
import de.rwth.montisim.simulation.simulator.randomization.RandomizationProperties;
import de.rwth.montisim.simulation.simulator.randomization.RandomizationStrategy;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.PathfinderRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.WorldRenderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.Control;
import de.rwth.montisim.simulation.simulator.visualization.ui.SimulationRunner;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.simulator.visualization.rl.RLVisualizer;

import javax.swing.*;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.*;
import java.awt.BorderLayout;
import java.io.File;
import java.time.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.Vector;
import java.util.stream.Collectors;

public class ScenarioVis extends SimVis implements SimulationRunner {
    private static final  long serialVersionUID = 7903217594061845406L;

    final FileSystem fileSystem;
    String current_scenario = "";
    JLabel scenario_name;
    JPanel topPanel;

    Control control;
    Viewer2D viewer;
    private List<CarRenderer> carRenderers = new ArrayList<>();

    Simulator simulator;
    SimulationConfig simConfig;
    World world;
    OsmMap map;
    Pathfinding pathfinding;
    boolean distributed = false;
    boolean randomize = false;
    boolean play = false;
    boolean miniStep = false;
    String selfPlay_mode = ".";

    final long PHYSICS_TICK_DURATION_MS = 10;
    final long TICK_NANO = PHYSICS_TICK_DURATION_MS * 1000000;
    Instant simTime = Instant.EPOCH;
    Duration dt = Duration.ofMillis(PHYSICS_TICK_DURATION_MS);

    public ScenarioVis(FileSystem fileSystem) {
        UIInfo.inspectAutopilots = false;
        // setBackground(Color.WHITE);
        this.fileSystem = fileSystem;

        setupUI();
    }

    private void setup() {
        // Cleanup if a sim is here
        if (simulator != null) {
            simulator.destroy();
            simulator = null;
            viewer.clearRenderers();
            carRenderers.clear();
        }

        File scenarioFile = fileSystem.getPath("scenarios", current_scenario + ".json");
        try {
            simConfig = SimulationConfig.fromFile(scenarioFile);
        } catch (SerializationException e1) {
            e1.printStackTrace();
            return;
        }

        // Randomize Scenario
		try {
			Vector<RandomizationProperties> randomizationPropertiesVector = simConfig.randomization;
			RandomizationProperties randomizationProperties = RandomRandomizationPropertiesPicker.pickRandomizationProperties(randomizationPropertiesVector);
			RandomizationStrategy strategy = randomizationProperties.build(simConfig, fileSystem.getPath("maps",  "").getAbsolutePath());
			simConfig.cars = strategy.randomizeCars(simConfig.cars);
			simConfig.map_name = strategy.randomizeMapName(simConfig.map_name);
			simConfig.max_duration = strategy.randomizeMaxDuration(simConfig.max_duration);
			simConfig.tick_duration = strategy.randomizeTickDuration(simConfig.tick_duration);
			simConfig.modules = strategy.randomizeModules(simConfig.modules);
		}catch (Exception e) {
			e.printStackTrace();
		}

        control.init(simConfig.tick_duration, simConfig.start_time);

        // Create simulator from scenario file

        File map_path = fileSystem.getPath("maps", simConfig.map_name + ".osm");
        World world;
        try {
            map = new OsmMap(simConfig.map_name, map_path);
            world = new OsmToWorldLoader(map).getWorld();
            pathfinding = new PathfindingImpl(world);
        } catch (Exception e1) {
            e1.printStackTrace();
            return;
        }
        if(current_scenario.charAt(0) == 'r' && current_scenario.charAt(1) == 'l'){
            topPanel.remove(control);
            RLVisualizer viz = new RLVisualizer(map, simConfig, viewer, simConfig.start_time);
            viz.init(distributed, randomize, play, miniStep, selfPlay_mode);
            return;
        }
        else{
            simulator = simConfig.build(world, pathfinding, map);
        }

        // Setup visualizer

        viewer.addRenderer(new WorldRenderer(world));
        viewer.addRenderer(new PathfinderRenderer(pathfinding));

        // Init CarRenderers and find view for all Vehicles
        Collection<Vehicle> vehicles = simulator.getVehicles().collect(Collectors.toCollection(ArrayList::new));
        setView(vehicles);

        for (Vehicle v : vehicles) {
            System.out.println(v.properties.vehicleName + " has a carrenderer");
            CarRenderer cr = new CarRenderer();
            cr.setCar(v);
            viewer.addRenderer(cr);
            carRenderers.add(cr);
        }

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

    @Override
    public void redraw() {
        for (CarRenderer cr : carRenderers)
            cr.dirty = true;
        viewer.update();
    }

    @Override
    public void reset() {
        setup();
    }

    @Override
    public void select(Category.Elem elem) {
        current_scenario = elem.name;
        setup();
    }

    @Override
    public void update(TimeUpdate newTime) {
        if (simulator != null)
            simulator.update(newTime);
    }

    void setupUI() {
        viewer = new Viewer2D();
        viewer.setZoom(20);

        control = new Control(Control.Mode.SIMULATION, Instant.EPOCH, this, Duration.ofMillis(PHYSICS_TICK_DURATION_MS), 30, 3);
        control.setBackground(Color.WHITE);

        topPanel = new JPanel();
        topPanel.setLayout(new BoxLayout(topPanel, BoxLayout.Y_AXIS));
        topPanel.setBorder(Browser.paneBorder);
        topPanel.add(control);

        setLayout(new BorderLayout());
        add(topPanel, BorderLayout.PAGE_START);

        viewer.setBackground(Color.WHITE);
        JPanel viewerContainer = new JPanel();
        viewerContainer.setLayout(new BoxLayout(viewerContainer, BoxLayout.Y_AXIS));
        viewerContainer.setBorder(Browser.paneBorder);
        viewerContainer.add(viewer);
        add(viewerContainer, BorderLayout.CENTER);

        JPanel interm = new JPanel();
        interm.setLayout(new FlowLayout());
        interm.setBackground(Color.WHITE);

        JCheckBox checkBox1 = new JCheckBox("Inspect Autopilot I/O", UIInfo.inspectAutopilots);
        checkBox1.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                UIInfo.inspectAutopilots = e.getStateChange() == 1;
                viewer.repaint();
            }
        });
        interm.add(checkBox1);

        JCheckBox checkBox2 = new JCheckBox("Show Planned Path", UIInfo.drawPlannedPath);
        checkBox2.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                UIInfo.drawPlannedPath = e.getStateChange() == 1;
                viewer.setDirty();
                viewer.repaint();
            }
        });
        interm.add(checkBox2);

        JCheckBox checkBox3 = new JCheckBox("Show Trajectory (Navigation output)", UIInfo.drawPlannedTrajectory);
        checkBox3.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                UIInfo.drawPlannedTrajectory = e.getStateChange() == 1;
                viewer.setDirty();
                viewer.repaint();
            }
        });
        interm.add(checkBox3);

        JCheckBox checkBox4 = new JCheckBox("Show Actuators", UIInfo.drawActuators);
        checkBox4.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                UIInfo.drawActuators = e.getStateChange() == 1;
                viewer.setDirty();
                viewer.repaint();
            }
        });
        interm.add(checkBox4);

        JCheckBox checkBox5 = new JCheckBox("Show driven Trajectory", UIInfo.drawDrivenTrajectory);
        checkBox5.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                UIInfo.drawDrivenTrajectory = e.getStateChange() == 1;
                viewer.setDirty();
                viewer.repaint();
            }
        });
        interm.add(checkBox5);

        JPanel bottomPanel = new JPanel();
        bottomPanel.setLayout(new BoxLayout(bottomPanel, BoxLayout.Y_AXIS));
        bottomPanel.setBorder(Browser.paneBorder);
        bottomPanel.add(interm);
        add(bottomPanel, BorderLayout.PAGE_END);

        // add(plotter, BorderLayout.PAGE_END);
    }

    @Override
    public TaskStatus status() {
        return simulator.status();
    }
}
