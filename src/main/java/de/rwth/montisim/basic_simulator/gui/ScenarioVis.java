/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eecomponents.navigation.Navigation;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.SimulationConfig;
import de.rwth.montisim.simulation.simulator.Simulator;
import de.rwth.montisim.simulation.simulator.visualization.car.CarRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.PathfinderRenderer;
import de.rwth.montisim.simulation.simulator.visualization.map.WorldRenderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.Control;
import de.rwth.montisim.simulation.simulator.visualization.ui.SimulationRunner;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

import javax.swing.*;
import java.awt.BorderLayout;
import java.io.File;
import java.time.*;
import java.util.ArrayList;
import java.util.List;

public class ScenarioVis extends SimVis implements SimulationRunner {
    private static final long serialVersionUID = 7903217594061845406L;

    public static final boolean SHOW_SEGMENTS = true;

    final FileSystem fileSystem;
    String current_scenario = "";
    JLabel scenario_name;

    final Control control;
    final Viewer2D viewer;
    private List<CarRenderer> carRenderers = new ArrayList<>();

    Simulator simulator;
    World world;
    Pathfinding pathfinding;
    MessageTypeManager mtManager;

    final long PHYSICS_TICK_DURATION_MS = 10;
    final long TICK_NANO = PHYSICS_TICK_DURATION_MS * 1000000;
    Instant simTime = Instant.EPOCH;
    Duration dt = Duration.ofMillis(PHYSICS_TICK_DURATION_MS);

    public ScenarioVis(FileSystem fileSystem) {
        this.fileSystem = fileSystem;

        viewer = new Viewer2D();
        viewer.setZoom(20);

        control = new Control(Control.Mode.SIMULATION, Instant.EPOCH, this, (int)PHYSICS_TICK_DURATION_MS, 30, 3);

        JPanel topPanel = new JPanel();
        topPanel.setLayout(new BoxLayout(topPanel, BoxLayout.Y_AXIS));
        topPanel.add(control);
        topPanel.add(new JSeparator());

        setLayout(new BorderLayout());
        add(topPanel, BorderLayout.PAGE_START);
        add(viewer, BorderLayout.CENTER);
        // add(plotter, BorderLayout.PAGE_END);
    }

    // public void setScenario(String scenario){
    // if (scenario != current_scenario){
    // current_scenario = scenario;
    // scenario_name.setText(scenario);
    // }
    // }

    private void setup() {
        File scenarioFile = fileSystem.getPath("scenarios", current_scenario + ".json");
        SimulationConfig config;
        try {
            config = SimulationConfig.fromFile(scenarioFile);
        } catch (SerializationException e1) {
            e1.printStackTrace();
            return;
        }

        // Create simulator from scenario file

        File map_path = fileSystem.getPath("maps", config.map_name + ".osm");
        World world;
        try {
            world = new OsmToWorldLoader(new OsmMap(config.map_name, map_path)).getWorld();
            pathfinding = new PathfindingImpl(world);
        } catch (Exception e1) {
            e1.printStackTrace();
            return;
        }
        mtManager = new MessageTypeManager();
        simulator = new Simulator(config, world, pathfinding, mtManager);

        // Setup visualizer

        viewer.clearRenderers();
        viewer.addRenderer(new WorldRenderer(world, SHOW_SEGMENTS));
        viewer.addRenderer(new PathfinderRenderer(pathfinding));

        carRenderers.clear();
        for (VehicleProperties car_config : config.cars) {
            Vehicle car;
            try {
                car = simulator.getVehicleBuilder(car_config).build();
                
                simulator.addSimulationObject(car);

                car.physicsModel.setGroundPosition(new Vec3(car_config.start_coords, 0), new Vec2(1,0));

                Navigation nav = (Navigation) car.eesystem.getComponentManager().getComponent("Navigation").get();
                nav.pushTargetPos(car_config.end_coords);
                car.addTarget(car_config.end_coords);

                CarRenderer cr = new CarRenderer();
                cr.setCar(car);
                viewer.addRenderer(cr);
                carRenderers.add(cr);
            } catch (SerializationException | EEMessageTypeException | EESetupException e) {
                e.printStackTrace();
            }
            
        }

        viewer.repaint();

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
}
