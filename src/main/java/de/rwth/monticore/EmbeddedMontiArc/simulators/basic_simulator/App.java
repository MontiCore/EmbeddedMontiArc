/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.BasicController;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui.Browser;

import javax.swing.*;
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
        //new HelloTriangleSimple().setup();
        //new Main().run(args);

        load_icons();
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
            //FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
            FileSystem fileSystem = new FileSystem(System.getProperty("user.dir") + "/");
            BasicController sim_controller = new BasicController(fileSystem);

            /* MapData map = new MapData(fileSystem.getPath("maps", "Aachen2.osm"));
            System.out.println("Map bounds: lat:["+map.minlat+" to "+map.maxlat+"] lon:["+map.minlon+" to "+map.maxlon+"]");
            System.out.println("Loaded " + map.nodes.size() +" nodes.");
            System.out.println("Loaded " + map.ways.size() +" ways."); */

            if (args.length > 0){
                String scenario_name = args[0];
                System.out.println("Starting simulation with scenario: " + scenario_name);
                sim_controller.initFromJsonScenario(FileSystem.getJson(fileSystem.getPath("scenarios", scenario_name + ".json")), scenario_name);
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
