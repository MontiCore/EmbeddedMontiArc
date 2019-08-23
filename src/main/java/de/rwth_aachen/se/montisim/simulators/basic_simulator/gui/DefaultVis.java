/* (c) https://github.com/MontiCore/monticore */
package de.rwth_aachen.se.montisim.simulators.basic_simulator.gui;

import javax.swing.*;

public class DefaultVis extends SimVis {
    public DefaultVis(){
        JButton b=new JButton("click");
        b.setBounds(130,100,100, 40);
        add(b);
    }

    @Override
    public void select(Category.Elem elem) {

    }
}
