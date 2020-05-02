package de.rwth.montisim.simulation.simulator.visualization.plotter;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import javax.swing.JPanel;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;

public class TimePlotter extends JPanel {
    private static final long serialVersionUID = -9135099086009538480L;
    
    
    public Vec2 screen_size = new Vec2();

    List<DataPoints> graphs = new ArrayList<>();

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        if (UIInfo.antialiasing) {
            RenderingHints rh = new RenderingHints(RenderingHints.KEY_TEXT_ANTIALIASING,
                    RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            rh.put(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHints(rh);
        }

        Dimension d = this.getSize();
        screen_size.x = d.getWidth();
        screen_size.y = d.getHeight();

        screen_size.y -= drawLegend(g2);
    }

    private int drawLegend(Graphics2D g) {
        // Get legend size
        final int height = g.getFont().getSize();
        final int offset = height + UIInfo.LINE_SPACE;
        int size = 0;
        // Draw legend
        return size;
    }

    public void addGraph(DataPoints dp) {
        graphs.add(dp);
    }
    
}