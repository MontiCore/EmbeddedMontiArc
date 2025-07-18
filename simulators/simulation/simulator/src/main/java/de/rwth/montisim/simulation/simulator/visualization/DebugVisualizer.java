/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization;

import javax.swing.JFrame;
import java.awt.BorderLayout;
import java.io.File;

import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.*;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.simulator.visualization.map.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.*;


/**
 * Start the main of this class to open a Map visualization. (MapViewer)
 */
public class DebugVisualizer extends JFrame {
    public static void main(String args[]) {
        new DebugVisualizer(args);
    }

    private static final long serialVersionUID = -2694724825418469027L;
    Viewer2D viewer;


    public DebugVisualizer(String args[]) {
        super("Debug Visualizer");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        setSize(900, 900);

        UIInfo.antialiasing = true;

        viewer = new Viewer2D();

        //vis3(args);
        vis1(args);

        setLayout(new BorderLayout());
        // add(new JLabel("Map Visualization."), BorderLayout.NORTH);
        add(viewer, BorderLayout.CENTER);

        setVisible(true);
    }

    public void vis1(String args[]) {
        System.out.println(LibraryService.getWorkingDirectory());
        String mapPath = "simulator/src/test/resources/aachen.osm";
        try {
            World world = new OsmToWorldLoader(new OsmMap("aachen", new File(mapPath))).getWorld();
            viewer.addRenderer(new WorldRenderer(world));
            viewer.addRenderer(new PathfinderRenderer(new PathfindingImpl(world)));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void vis2(String args[]) {
        String mapPath = "simulator/src/test/resources/Aachen2.osm";
        try {
            World world = new OsmToWorldLoader(new OsmMap("aachen", new File(mapPath))).getWorld();
            viewer.addRenderer(new WorldRenderer(world));
            viewer.addRenderer(new PathfinderRenderer(new PathfindingImpl(world)));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void vis3(String args[]) {
        World world = generateSimpleWorld();
        viewer.addRenderer(new WorldRenderer(world));
        viewer.addRenderer(new PathfinderRenderer(new PathfindingImpl(world)));
    }


    public static World generateSimpleWorld() {
        World world = new World("SimpleTestWorld");

        Building bd = new Building("TestBuilding", 0, "Test", 10, 3);
        bd.boundary.add(new Vec3(-1, -5, 0));
        bd.boundary.add(new Vec3(-1, 10, 0));
        bd.boundary.add(new Vec3(-10, 10, 0));
        bd.boundary.add(new Vec3(-15, -5, 0));
        world.buildings.add(bd);


        Vec3 a = new Vec3(2, 1, 0);
        Vec3 b = new Vec3(3, 1, 0);
        Vec3 c = new Vec3(2, 2, 0);

        int aid = world.addNode(new Node(a));
        int bid = world.addNode(new Node(b));
        int cid = world.addNode(new Node(c));

        Way r1 = new Way("r1", false, 1, false, 50);
        r1.addPoint(new Vec3(0, 0, 0), -1);
        r1.addPoint(new Vec3(1, 0, 0), -1);
        r1.addPoint(new Vec3(1, 1, 0), -1);
        r1.addPoint(a, aid);
        r1.addPoint(new Vec3(2.5, 1, 0), -1);
        r1.addPoint(b, bid);
        r1.addPoint(new Vec3(4, 1, 0), -1);

        Way r2 = new Way("r2", false, 1, false, 50);
        r2.addPoint(new Vec3(5, 0, 0), -1);
        r2.addPoint(new Vec3(4, 0, 0), -1);
        r2.addPoint(b, bid);
        r2.addPoint(new Vec3(3, 3, 0), -1);
        r2.addPoint(new Vec3(3, 4, 0), -1);
        r2.addPoint(new Vec3(3, 5, 0), -1);

        Way r3 = new Way("r3", false, 1, false, 50);
        r3.addPoint(a, aid);
        r3.addPoint(c, cid);
        r3.addPoint(new Vec3(1, 2, 0), -1);

        Way r4 = new Way("r4", true, 1, false, 50);
        r4.addPoint(c, cid);
        r4.addPoint(b, bid);

        world.addWay(r1);
        world.addWay(r2);
        world.addWay(r3);
        world.addWay(r4);

        world.finalizeGraph();

        return world;
    }

}