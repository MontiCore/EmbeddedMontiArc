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

        FileSystem fileSystem = new FileSystem(LibraryService.getWorkingDirectory());
        BasicController sim_controller = new BasicController(fileSystem);
        sim_controller.initFromJsonScenario(FileSystem.getJson(fileSystem.getPath(Category.CategoryType.SCENARIOS.id, "straight.json")));
        sim_controller.startSimulation();
    }
}
