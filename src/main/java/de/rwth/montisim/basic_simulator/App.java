/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.basic_simulator.gui.Browser;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.TaskStatus;
import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.simulator.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;

import javax.swing.*;

import java.io.File;
import java.io.IOException;

public class App 
{
    static {
        TypedHardwareEmu.registerTypedHardwareEmu();
    }

    public static void main( String[] args )
    {
        UIInfo.antialiasing = true;
        /* System.out.println("Looks and Feels:");
        UIManager.LookAndFeelInfo[] looks = UIManager.getInstalledLookAndFeels();
        for (UIManager.LookAndFeelInfo look : looks) {
            System.out.println(look.getClassName());
        } */
        try {
            /*
                javax.swing.plaf.metal.MetalLookAndFeel
                javax.swing.plaf.nimbus.NimbusLookAndFeel
                com.sun.java.swing.plaf.motif.MotifLookAndFeel
                com.sun.java.swing.plaf.gtk.GTKLookAndFeel
                com.sun.java.swing.plaf.windows.WindowsLookAndFeel
            */
            UIManager.setLookAndFeel("javax.swing.plaf.nimbus.NimbusLookAndFeel");
        } catch (UnsupportedLookAndFeelException ex) {
            ex.printStackTrace();
        } catch (IllegalAccessException ex) {
            ex.printStackTrace();
        } catch (InstantiationException ex) {
            ex.printStackTrace();
        } catch (ClassNotFoundException ex) {
            ex.printStackTrace();
        }


        try {

            CppBridge.init("{\"softwares_folder\": \"autopilots\"}");
            FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());

            if (args.length > 0){
                String scenario_name = args[0];
                System.out.println("Starting simulation with scenario: " + scenario_name);
                runSimulation(scenario_name);
                return;
            }

            new Browser(fileSystem);
            fileSystem.check_updates(); //Infinite loop until gui calls System.exit()
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    protected static void runSimulation(String path){
        try {
            // Create simulator from scenario file
            File scenarioFile = new File(path);
            SimulationConfig config = SimulationConfig.fromFile(scenarioFile);
            File mapPath = new File(config.map_name + ".osm");
            OsmMap map = new OsmMap(config.map_name, mapPath);
            World world = new OsmToWorldLoader(map).getWorld();
            Pathfinding pathfinding = new PathfindingImpl(world);
            Simulator simulator = config.build(world, pathfinding, map);

            // Run simulation
            SimulationLoop simLoop = new SimulationLoop(simulator, config);
            TaskStatus res = simLoop.run();
            if (res == TaskStatus.SUCCEEDED) {
                System.out.println("Simulation SUCCEEDED.");
            } else {
                System.out.println("Simulation FAILED.");
                System.exit(-1);
            }
        } catch (Exception e1) {
            e1.printStackTrace();
            return;
        }
    }

    protected static ImageIcon createImageIcon(String path) {
        java.net.URL imgURL = App.class.getResource(path);
        //TODO error handling ?
        return new ImageIcon(imgURL);
    }
}
