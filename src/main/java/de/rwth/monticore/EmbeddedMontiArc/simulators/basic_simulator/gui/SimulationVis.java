/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui;

import javax.swing.*;

public class SimulationVis extends SimVis {
    public SimulationVis(){
        add(new JLabel("Coming: Live visualization of the active simulations"));
    }

    @Override
    public void select(Category.Elem elem) {

    }
}
