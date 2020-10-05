/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import java.awt.BorderLayout;
import java.io.File;

import javax.swing.*;

import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.visualization.map.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;

public class MapVis extends SimVis {
    public static final boolean SHOW_SEGMENTS = true;

    Viewer2D viewer;
    FileSystem fileSystem;

    public MapVis(FileSystem fileSystem){
        this.fileSystem = fileSystem;
        setLayout(new BorderLayout());
        add(new JLabel("Visualization of available maps."), BorderLayout.NORTH);
        viewer = new Viewer2D();
        add( viewer, BorderLayout.CENTER );
    }

    @Override
    public void select(Category.Elem elem) {
        try {
            File map_path = fileSystem.getPath("maps", elem.name + ".osm");
            World world = new OsmToWorldLoader(new OsmMap("aachen", map_path)).getWorld();
            viewer.clearRenderers();
            viewer.addRenderer(new WorldRenderer(world, SHOW_SEGMENTS));
            viewer.addRenderer(new PathfinderRenderer(new PathfindingImpl(world)));
            viewer.repaint();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
