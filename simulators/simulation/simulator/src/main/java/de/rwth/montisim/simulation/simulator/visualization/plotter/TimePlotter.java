/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.plotter;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.simulator.visualization.ui.UIInfo;

public class TimePlotter extends JPanel {
    private static final long serialVersionUID = -9135099086009538480L;

    public static final int LEGEND_LINE_WIDTH = 12;
    public static final int LEGEND_LINE_HEIGHT = 3;

    public boolean dirty = true;

    public Vec2 screen_size = new Vec2();

    List<DataPoints> graphs = new ArrayList<>();

    public TimePlotter() {
        setPreferredSize(new Dimension(100, 300));
    }

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

        g2.setColor(new Color(255, 180, 180));
        g2.fillRect(0, 0, (int) d.getWidth(), (int) d.getHeight());

        if (dirty) {
            refresh();
        }

        screen_size.y -= drawLegend(g2);

        drawAxesAndGrid();

        drawGraphs();
    }


    public void update() {
        // Update time for graphs
        for (DataPoints dp : graphs) {
            dp.nextTimeStep();
        }
        dirty = true;
    }

    private void refresh() {
        // TODO
        // Pre-process data point ranges:
        // - get minimum required y display range
        // - Clamp to "grid"
        // - Get axes texts & sizes + tick spacing (see map grid?)
        // Pre-compute graphs geometry (depending on scale AND scroll)
        // Only add new Graph Point when distance to previous > threshold
        dirty = false;
    }

    private int drawLegend(Graphics2D g) {
        // Get legend size
        //final int height = g.getFont().getSize();
        //final int offset = height + UIInfo.LINE_SPACE;
        int size = 0;
        // Draw legend
        return size;
    }


    private void drawAxesAndGrid() {
        // Use precomputed labels sizes to draw axes lines
        // Use precomputed info to draw label ticks and labels
        // Draw Grid
    }

    private void drawGraphs() {
        // Set clip region
        // For all graphs
        // Draw geom
        // If big scale -> draw points
        // If current time visible -> Draw Vertical Line
    }


    public void addGraph(DataPoints dp) {
        graphs.add(dp);
        dirty = true;
    }

}