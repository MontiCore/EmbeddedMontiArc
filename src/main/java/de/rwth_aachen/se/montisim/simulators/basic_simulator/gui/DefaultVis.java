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
