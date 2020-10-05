/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.basic_simulator.gui.Browser;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.commons.utils.json.*;
import de.rwth.montisim.hardware_emulator.*;
import de.rwth.montisim.simulation.eecomponents.autopilots.*;
import de.rwth.montisim.simulation.eecomponents.navigation.NavigationProperties;
import de.rwth.montisim.simulation.eesimulator.actuator.ActuatorProperties;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.can.CANProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestCompProperties;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.osmmap.*;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.simulator.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.fuel.FuelPTProperties;

import javax.swing.*;

import java.io.File;
import java.io.IOException;

public class App 
{
    static {
        try {
            Json.registerType(ElectricalPTProperties.class);
            Json.registerType(FuelPTProperties.class);
            Json.registerType(NavigationProperties.class);
            Json.registerType(JavaAutopilotProperties.class);
            Json.registerType(TestAutopilotProperties.class);
            Json.registerType(RigidbodyPhysicsProperties.class);
            Json.registerType(ActuatorProperties.class);
            Json.registerType(BridgeProperties.class);
            Json.registerType(CANProperties.class);
            Json.registerType(ConstantBusProperties.class);
            Json.registerType(SensorProperties.class);
            Json.registerType(TestCompProperties.class);
            Json.registerType(ComputerProperties.class);
        } catch (SerializationException e) {
            e.printStackTrace();
            System.exit(-1);
        }
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

            Browser gui = new Browser(fileSystem);
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
            World world = new OsmToWorldLoader(new OsmMap(config.map_name, mapPath)).getWorld();
            Pathfinding pathfinding = new PathfindingImpl(world);
            MessageTypeManager mtManager = new MessageTypeManager();
            Simulator simulator = new Simulator(config, world, pathfinding, mtManager);

            // Run simulation
            SimulationLoop simLoop = new SimulationLoop(simulator, config);
            simLoop.run();
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
