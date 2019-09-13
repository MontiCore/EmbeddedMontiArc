/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui;

import java.awt.BorderLayout;

import javax.swing.*;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization.MapModel;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization.Visualizer;

public class MapVis extends SimVis {
    Visualizer vis;
    FileSystem fileSystem;

    public MapVis(FileSystem fileSystem){
        this.fileSystem = fileSystem;
        setLayout(new BorderLayout());
        add(new JLabel("Visualization of available maps."), BorderLayout.NORTH);
        vis = new Visualizer(false);
        add( vis, BorderLayout.CENTER );
    }

    @Override
    public void select(Category.Elem elem) {
        try {
            MapData map = new MapData(elem.name, fileSystem);
            vis.map_model = new MapModel(map);
            vis.repaint();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
