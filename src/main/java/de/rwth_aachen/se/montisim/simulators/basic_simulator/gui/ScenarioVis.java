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
