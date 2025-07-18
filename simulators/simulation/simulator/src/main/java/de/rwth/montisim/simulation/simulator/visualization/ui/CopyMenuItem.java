/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import javax.swing.JMenuItem;

import java.awt.event.*;
import java.awt.datatransfer.*;
import java.awt.Toolkit;

public class CopyMenuItem extends JMenuItem implements ActionListener {
    private static final long serialVersionUID = 170035139576795954L;

    final String textToCopy;

    // Preview: wether the text to be copied is shown in parenthesis after the button message
    public CopyMenuItem(String msg, String textToCopy, boolean preview) {
        super(preview ? msg + " (" + textToCopy + ")" : msg);
        this.textToCopy = textToCopy;
        addActionListener(this);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        StringSelection stringSelection = new StringSelection(textToCopy);
        Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
        clipboard.setContents(stringSelection, null);
    }

}