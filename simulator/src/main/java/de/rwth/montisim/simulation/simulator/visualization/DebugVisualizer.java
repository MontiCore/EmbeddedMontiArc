/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.JFrame;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.map.Map;
import de.rwth.montisim.simulation.environment.map.elements.Building;
import de.rwth.montisim.simulation.environment.map.elements.Road;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToMapConverter;
import de.rwth.montisim.simulation.simulator.visualization.map.MapRenderer;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;
import de.rwth.montisim.simulation.simulator.visualization.ui.Viewer2D;

import java.awt.BorderLayout;
import java.io.File;

/**
 * Start the main of this class to open a Map visualization. (MapViewer)
 */
public class DebugVisualizer extends JFrame {
    private static final long serialVersionUID = -2694724825418469027L;
    Viewer2D viewer;


    public DebugVisualizer(String args[]) {
        super("Debug Visualizer");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(900, 900);

        UIInfo.antialiasing = true;

        viewer = new Viewer2D();

        //vis1(args);

        setLayout(new BorderLayout());
        // add(new JLabel("Map Visualization."), BorderLayout.NORTH);
        add(viewer, BorderLayout.CENTER);

        setVisible(true);
    }

    public void vis1(String args[]) {
        String mapPath = "D:/EmbededMontiArc/basic-simulator/install/maps/aachen.osm";
        try {
            Map map = new OsmToMapConverter(new OsmMap("aachen", new File(mapPath))).getMap();
            viewer.addRenderer(new MapRenderer(map));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void vis2(String args[]) {
        String mapPath = "D:/EmbededMontiArc/basic-simulator/install/maps/Aachen2.osm";
        try {
            Map map = new OsmToMapConverter(new OsmMap("aachen", new File(mapPath))).getMap();
            viewer.addRenderer(new MapRenderer(map));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void vis3(String args[]) {
        Map map = generateSampleMap();
        viewer.addRenderer(new MapRenderer(map));
    }

    public static Map generateSampleMap() {
        Map map = new Map("GeneratedTestMap");
        Road r = new Road("TestRoad", true, 1, false);
        r.points.add(new Vec3(0, 0, 0));
        r.points.add(new Vec3(100, 0, 0));
        r.points.add(new Vec3(100, 50, 0));
        r.points.add(new Vec3(115, 62, 0));
        map.roads.add(r);
        Building b = new Building("TestBuilding", "Test", 10, 3);
        b.boundary.add(new Vec2(-1, -5));
        b.boundary.add(new Vec2(-1, 10));
        b.boundary.add(new Vec2(-10, 10));
        b.boundary.add(new Vec2(-15, -5));
        map.buildings.add(b);
        return map;
    }


}