/**
 * (c) https://github.com/MontiCore/monticore
 */
package de.rwth.montisim.basic_simulator.gui;

import javax.swing.*;

public class DefaultVis extends SimVis {
    public DefaultVis(){
        setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
        setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        add_text("Select one of the scenarios to start a simulation.");
        add_text("Watch completed simulations in the results. (Coming)");
        
        /* System.out.println("Creating OpenGL context.");
        GLProfile glprofile = GLProfile.getDefault();
        GLCapabilities glcapabilities = new GLCapabilities( glprofile );
        add(new OpenGLPanel(glcapabilities)); */
    }

    private void add_text(String text){
        JLabel t = new JLabel(text);
        add(t);
    }

    @Override
    public void select(Category.Elem elem) {

    }
}
