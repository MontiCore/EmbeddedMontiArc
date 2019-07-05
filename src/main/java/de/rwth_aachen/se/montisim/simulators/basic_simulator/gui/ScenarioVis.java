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

import de.rwth_aachen.se.montisim.simulators.App;

import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class ScenarioVis extends SimVis implements ActionListener {

    Browser browser;
    String current_scenario = "";
    JLabel scenario_name;

    public ScenarioVis(Browser browser){
        this.browser = browser;
        JLabel s = new JLabel("Scenario: ");
        add(s);
        scenario_name = new JLabel("");
        add(scenario_name);
        JButton start_button = new JButton("", App.play_icon);
        start_button.setActionCommand("start_simulation");
        start_button.addActionListener(this);
        start_button.setToolTipText("Start a simulation with this scenario.");
        add(start_button);
    }

    public void setScenario(String scenario){
        if (scenario != current_scenario){
            current_scenario = scenario;
            scenario_name.setText(scenario);
        }
    }


    @Override
    public void actionPerformed(ActionEvent e) {
        if ("start_simulation".equals(e.getActionCommand())) {
            //TODO
            browser.start_simulation(current_scenario);
            JOptionPane.showMessageDialog(this, "Starting simulation " + current_scenario);
        }
    }

    @Override
    public void select(Category.Elem elem) {
        setScenario(elem.name);
    }
}
