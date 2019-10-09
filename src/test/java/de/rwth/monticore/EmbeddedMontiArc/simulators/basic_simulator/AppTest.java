/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator;

import static org.junit.Assert.assertTrue;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.LibraryService;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.BasicController;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui.Category;
import org.junit.Test;

import java.io.IOException;


public class AppTest 
{
    @Test
    public void basicSetupTest() throws Exception {

        FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
        BasicController sim_controller = new BasicController(fileSystem);
        sim_controller.initFromJsonScenario(FileSystem.getJson(fileSystem.getPath(Category.CategoryType.SCENARIOS.id, "straight.json")), "straight");
        sim_controller.startSimulation();
    }
}
