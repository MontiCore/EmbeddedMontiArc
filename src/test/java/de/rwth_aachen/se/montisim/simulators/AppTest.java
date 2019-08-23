/* (c) https://github.com/MontiCore/monticore */
package de.rwth_aachen.se.montisim.simulators;

import static org.junit.Assert.assertTrue;

import commons.utils.LibraryService;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.controller.BasicController;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth_aachen.se.montisim.simulators.basic_simulator.gui.Category;
import org.junit.Test;

import java.io.IOException;


public class AppTest 
{
    @Test
    public void basicSetupTest() throws Exception {

        //FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
        FileSystem fileSystem = new FileSystem(System.getProperty("user.dir") + "/");
        BasicController sim_controller = new BasicController(fileSystem);
        sim_controller.initFromJsonScenario(FileSystem.getJson(fileSystem.getPath(Category.CategoryType.SCENARIOS.id, "straight.json")));
        sim_controller.startSimulation();
    }
}
