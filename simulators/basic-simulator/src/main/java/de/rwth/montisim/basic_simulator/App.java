/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.basic_simulator.gui.Browser;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.simulation.commons.TaskStatus;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.osmmap.OsmToWorldLoader;
import de.rwth.montisim.simulation.environment.pathfinding.PathfindingImpl;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;

import java.io.File;
import java.io.IOException;

import javax.swing.*;

public class App 
{

    public static void main( String[] args ) throws Exception
    {
        TypedHardwareEmu.registerTypedHardwareEmu();
        initUISettings();

        try {

            CppBridge.init("{\"softwares_folder\": \"autopilots\"}");
            FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
            
            String scenario_name = null;
            boolean reinforcement_learning = false;
            boolean decentralized = false;
            boolean randomize = false;
            boolean play = false;
            boolean miniStep = false;
            String selfPlay_mode = ".";
            String maps_folder = ".";

            if(args.length > 0){
                if(args.length == 1 && (args[0].equals("-h") || args[0].equals("--help"))){
                    //print argument help for every cli option
                    System.out.println("usage: ./run.sh [options]\n");
                    System.out.println("Options:");
                    System.out.printf(" %-30s %-20s\n", "-h,--help" ,"Display help information");
                    System.out.printf(" %-30s %-20s\n", "-m,--maps <folder>" ,"Specify the maps folder");
                    System.out.printf(" %-30s %-20s\n", "-d,--decentralized" ,"Use decentralized mode. Only relevant for Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-mS,--mini_step" ,"Use Mini-Step mode. Only relevant for Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-nU,--no_update" ,"Do not update the trained vehicle. Only available for decentralized mode in Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-aS,--after_step" ,"Update the trained vehicle after every trainings step. Only relevant for Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-aE,--after_episode" ,"Update the trained vehicle after every episode. Only available for decentralized mode in Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-rl,--reinforcement_learning" ,"Use simulator for Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-s,--scenario <arg>" ,"Relative path to executed scenario");
                    System.out.printf(" %-30s %-20s\n", "-r,--randomize" ,"Randomize the scenarios. Only relevant for Reinforcement Learning");
                    System.out.printf(" %-30s %-20s\n", "-p,--play" ,"Use the simulator for playing and not training of an RL agent. Only relevant for Reinforcement Learning");
                    System.out.println("");
                    return;
                }
                for(int i = 0; i<args.length; i++){
                    if(args[i].equals("-rl") || args[i].equals("--reinforcement_learning")){
                        reinforcement_learning = true;
                    }

                    if(args[i].equals("-s") || args[i].equals("--scenario")){
                        if(scenario_name != null){
                            System.out.println("Please give only one scenario. Use '--help' for more information.");
                            System.exit(-1);
                            return;
                        }
                        if(i+1 >= args.length){
                            System.out.println("Please provide a scenario. Use '--help' for more information.");
                            System.exit(-1);
                            return;
                        }
                        ++i;
                        scenario_name = args[i];
                    }
                    
                    if(args[i].equals("-m") || args[i].equals("--maps")){
                        if(i+1 >= args.length){
                            System.out.println("Please provide a maps folder path.");
                            System.exit(-1);
                            return;
                        }
                        ++i;
                        maps_folder = args[i];
                    }

                    if(args[i].equals("-d") || args[i].equals("--decentralized")){
                        decentralized = true;
                    }

                    if(args[i].equals("-mS") || args[i].equals("--mini_step")){
                        miniStep = true;
                    }

                    if(args[i].equals("-nU") || args[i].equals("--no_update")){
                        selfPlay_mode = "noUpdate";
                        if(!decentralized) {
                            System.out.println("Please use the decentralized mode for the option no_update. Use '--help' for more information.");
                        }
                        if(selfPlay_mode == "afterStep" || selfPlay_mode == "afterEpisode") {
                            System.out.println("Please only use one self play option. Use '--help' for more information.");
                        }
                    }

                    if(args[i].equals("-aS") || args[i].equals("--after_step")){
                        selfPlay_mode = "afterStep";
                        if(!decentralized) {
                            System.out.println("Please use the decentralized mode for the option after_step. Use '--help' for more information.");
                        }

                        if(selfPlay_mode == "noUpdate" || selfPlay_mode == "afterEpisode") {
                            System.out.println("Please only use one self play option. Use '--help' for more information.");
                        }
                    }

                    if(args[i].equals("-aE") || args[i].equals("--after_episode")){
                        selfPlay_mode = "afterEpisode";
                        if(!decentralized) {
                            System.out.println("Please use the decentralized mode for the option after_episode. Use '--help' for more information.");
                        }
                        
                        if(selfPlay_mode == "noUpdate" || selfPlay_mode == "afterStep") {
                            System.out.println("Please only use one self play option. Use '--help' for more information.");
                        }
                    }

                    if(args[i].equals("-r") || args[i].equals("--randomize")){
                        randomize = true;
                    }

                    if(args[i].equals("-p") || args[i].equals("--play")){
                        play = true;
                    }

                }
                if(scenario_name == null){
                    System.out.println("Please provide a scenario.");
                    System.exit(-1);
                    return;
                }
                if(reinforcement_learning){
                    System.out.print("\nStarting reinforcement learning simulation with scenario: " + scenario_name + ".");
                    System.out.println(" Currently used settings:");
                    if(decentralized){
                        System.out.print("Decentralized mode.\n");
                        if(selfPlay_mode == "noUpdate") {
                            System.out.print("Self play mode: No update.\n");
                        } else if(selfPlay_mode == "afterStep") {
                            System.out.print("Self play mode: Update after step.\n");
                        } else if(selfPlay_mode == "afterEpisode") {
                            System.out.print("Self play mode: Update after episode.\n");
                        }
                    }
                    else if(miniStep) {
                        System.out.print("Mini-Step mode.\n");
                    }
                    else{
                        System.out.print("Centralized mode. \n");
                    }
                    if(randomize){
                        System.out.println("Scenario randomization active");
                    }
                    else{
                        System.out.println("Scenario randomization inactive");
                    }
                    if(play){
                        System.out.println("Using PLAY mode");
                    }
                    else{
                        System.out.println("Using training mode");
                    }
                    runRLSimulation(scenario_name, decentralized, randomize, play, miniStep, selfPlay_mode);
                }
                else{
                    System.out.println("Starting simulation with scenario: " + scenario_name);
                    TaskStatus res = SimulationCLI.runSimulationFromFile(scenario_name, maps_folder);
                    if (res != TaskStatus.SUCCEEDED) 
                        System.exit(-1);
                }
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




    protected static void runRLSimulation(String path, boolean distributed, boolean randomize, boolean play, boolean miniStep, String selfPlay_mode){
        String lib_path = System.getProperty("user.dir") + "/";
        System.load(lib_path + "libROSInterface.so");
        try {
            File scenarioFile = new File(path);
            SimulationConfig config = SimulationConfig.fromFile(scenarioFile);
            File mapPath = new File(config.map_name + ".osm");
            OsmMap map = new OsmMap(config.map_name, mapPath);

            //Run Simulation
            RLSimulationInit simInit = new RLSimulationInit(config, map);
            simInit.setRLSettings(distributed, randomize, play, miniStep, selfPlay_mode);
            simInit.init();
        } catch (Exception e1) {
            e1.printStackTrace();
            return;
        }
    }

    
    static void initUISettings() {
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


    }

    protected static ImageIcon createImageIcon(String path) {
        java.net.URL imgURL = App.class.getResource(path);
        //TODO error handling ?
        return new ImageIcon(imgURL);
    }
}
