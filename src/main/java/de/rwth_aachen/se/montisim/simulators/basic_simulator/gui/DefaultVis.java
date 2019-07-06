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
package de.rwth_aachen.se.montisim.simulators.basic_simulator.gui;

import javax.swing.*;

public class DefaultVis extends SimVis {
    public DefaultVis(){
        setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
        setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        add_text("Select one of the scenarios to start a simulation.");
        add_text("Visualize the available maps. (Coming)");
        add_text("Watch completed simulations in the results. (Coming)");
    }

    private void add_text(String text){
        JLabel t = new JLabel(text);
        add(t);
    }

    @Override
    public void select(Category.Elem elem) {

    }
}
