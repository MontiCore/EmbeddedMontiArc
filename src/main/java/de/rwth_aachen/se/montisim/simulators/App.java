package de.rwth_aachen.se.montisim.simulators;

import commons.utils.LibraryService;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.controller.BasicController;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.gui.Browser;

import javax.swing.*;
import java.io.File;
import java.io.IOException;

/**
 * Hello world!
 *
 */
public class App 
{
    public static ImageIcon play_icon;
    public static ImageIcon stop_icon;
    public static ImageIcon pause_icon;
    public static void load_icons(){
        play_icon = createImageIcon("/images/play-solid.png");
        stop_icon = createImageIcon("/images/stop-solid.png");
        pause_icon = createImageIcon("/images/pause-solid.png");
    }

    public static void main( String[] args )
    {
        load_icons();
        try {
            UIManager.setLookAndFeel("com.sun.java.swing.plaf.windows.WindowsLookAndFeel");
            //UIManager.setLookAndFeel("javax.swing.plaf.metal.MetalLookAndFeel");
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
            //FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
            FileSystem fileSystem = new FileSystem(System.getProperty("user.dir") + "/");
            BasicController sim_controller = new BasicController(fileSystem);

            if (args.length > 0){
                String scenario_name = args[0];
                System.out.println("Starting simulation with scenario: " + scenario_name);
                sim_controller.initFromJsonScenario(FileSystem.getJson(fileSystem.getPath("scenarios", scenario_name + ".json")));
                sim_controller.startSimulation();
                return;
            }
            Browser gui = new Browser(fileSystem, sim_controller);
            fileSystem.check_updates(); //Infinite loop until gui calls System.exit()
        } catch (IOException e) {
            e.printStackTrace();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    protected static ImageIcon createImageIcon(String path) {
        java.net.URL imgURL = App.class.getResource(path);
        //TODO error handling ?
        return new ImageIcon(imgURL);
    }
}
