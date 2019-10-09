/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui;

import java.awt.BorderLayout;

import javax.swing.*;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimulationResult;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization.MapModel;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization.Visualizer;

public class ResultVis extends SimVis {
    SimulationResult result;
    
    Visualizer vis;
    FileSystem file_system;

    public ResultVis(FileSystem file_system) {
        setLayout(new BorderLayout());
        add(new JLabel("Coming: Visualization of completed simulations"), BorderLayout.NORTH);
        vis = new Visualizer(true);
        add( vis, BorderLayout.CENTER );
        result = new SimulationResult(file_system);
        this.file_system = file_system;
    }

    @Override
    public void select(Category.Elem elem) {
        try {
            result.import_result(elem.name);
            vis.simulation_result = result;
            vis.map_model = new MapModel(new MapData(result.map, file_system));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
