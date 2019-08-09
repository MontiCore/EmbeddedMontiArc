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
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui;

import java.awt.BorderLayout;

import javax.swing.*;

import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller.SimulationResult;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.FileSystem;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.filesystem.MapData;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization.MapModel;
import de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization.Visualizer;

public class ResultVis extends SimVis {
    SimulationResult result;
    
    Visualizer vis;
    FileSystem file_system;

    public ResultVis(FileSystem file_system) {
        setLayout(new BorderLayout());
        add(new JLabel("Coming: Visualization of completed simulations"), BorderLayout.NORTH);
        vis = new Visualizer(true);
        add( vis, BorderLayout.CENTER );
        result = new SimulationResult(file_system);
        this.file_system = file_system;
    }

    @Override
    public void select(Category.Elem elem) {
        try {
            result.import_result(elem.name);
            vis.simulation_result = result;
            vis.map_model = new MapModel(new MapData(result.map, file_system));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
