/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator;

import de.rwth.montisim.basic_simulator.filesystem.FileSystem;
import de.rwth.montisim.basic_simulator.gui.Browser;
import de.rwth.montisim.commons.utils.LibraryService;
import de.rwth.montisim.hardware_emulator.CppBridge;
import de.rwth.montisim.hardware_emulator.TypedHardwareEmu;
import de.rwth.montisim.simulation.simulator.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;

import javax.swing.*;

public class App 
{

    public static void main( String[] args ) throws Exception
    {
        TypedHardwareEmu.registerTypedHardwareEmu();
        initUISettings();
        CppBridge.init("{\"softwares_folder\": \"autopilots\"}");

        if (args.length > 0){
            // CLI Mode (no UI)
            String scenario_name = args[0];
            System.out.println("Starting simulation with scenario: " + scenario_name);
            SimulationCLI.runSimulationFromFile(scenario_name);

        } else {

            FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
            new Browser(fileSystem);
            fileSystem.check_updates(); //Infinite loop until gui calls System.exit()
    
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
