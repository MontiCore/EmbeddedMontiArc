/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.gui;

import javax.swing.*;

public abstract class SimVis extends JPanel{
    String id;
    public String getId(){
        return id;
    }
    public void setId(String id){
        this.id = id;
    }
    public abstract void select(Category.Elem elem);
}
