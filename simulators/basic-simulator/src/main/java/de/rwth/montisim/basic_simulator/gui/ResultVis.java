/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import java.awt.BorderLayout;

import javax.swing.*;

//import de.rwth.montisim.basic_simulator.controller.SimulationResult;
import de.rwth.montisim.basic_simulator.filesystem.FileSystem;

public class ResultVis extends SimVis {
    //SimulationResult result;
    
    //Visualizer vis;
    FileSystem file_system;

    public ResultVis(FileSystem file_system) {
        setLayout(new BorderLayout());
        add(new JLabel("Coming: Visualization of completed simulations"), BorderLayout.NORTH);
        // vis = new Visualizer(true);
        // add( vis, BorderLayout.CENTER );
        // result = new SimulationResult(file_system);
        // this.file_system = file_system;
    }

    @Override
    public void select(Category.Elem elem) {
        // try {
        //     result.import_result(elem.name);
        //     vis.simulation_result = result;
        //     vis.map_model = new MapModel(new MapData(result.map, file_system));
        // } catch (Exception e) {
        //     e.printStackTrace();
        // }
    }
}
