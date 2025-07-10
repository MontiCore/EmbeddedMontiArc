/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import javax.swing.*;

public abstract class SimVis extends JPanel {
    String id;
    public String getId(){
        return id;
    }
    public void setId(String id){
        this.id = id;
    }
    public abstract void select(Category.Elem elem);
}
