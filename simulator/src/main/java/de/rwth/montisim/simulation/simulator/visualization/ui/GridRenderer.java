/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import java.awt.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.swing.JMenuItem;

import de.rwth.montisim.commons.utils.*;

public class GridRenderer extends Renderer {
    private static final int MIN_GRID_SPACING_PX = 20;
    public static final Color GRID_COLOR = new Color(215, 215, 215);

    final Viewer2D viewer;

    public boolean drawGrid = true;
    // World size of grid cells in meters
    private double gridRes = 1;

    public GridRenderer(Viewer2D viewer) {
        this.viewer = viewer;
    }

    @Override
    public void draw(Graphics2D g) {
        Vec2 top_left = viewer.getWorldPos(new Vec2(0, 0));
        Vec2 bottom_right = viewer.getWorldPos(viewer.screen_size);
        Vec2 grid_space = top_left.multiply(1 / gridRes);
        grid_space.x = Math.ceil(grid_space.x);
        grid_space.y = Math.floor(grid_space.y);
        g.setColor(GRID_COLOR);
        g.setStroke(new BasicStroke(1));
        int height = (int) viewer.screen_size.y;
        for (double x = grid_space.x * gridRes; x <= bottom_right.x; x += gridRes) {
            Vec3 r = viewer.viewMatrix.multiply(new Vec3(x, 0, 1));
            int xi = (int) Math.round(r.x);
            g.drawLine(xi, 0, xi, height);
        }
        int width = (int) viewer.screen_size.x;
        for (double y = grid_space.y * gridRes; y >= bottom_right.y; y -= gridRes) {
            Vec3 r = viewer.viewMatrix.multiply(new Vec3(0, y, 1));
            int yi = (int) Math.round(r.y);
            g.drawLine(0, yi, width, yi);
        }
    }

    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        double minWorldSpacing = MIN_GRID_SPACING_PX / viewer.scale;
        double size = 1;
        gridRes = 1;
        int i = 0;
        // If the start grid size if too big
        while (size >= minWorldSpacing) {
            gridRes = size;
            if (i % 3 == 0) {
                size *= 0.5;
            } else if (i % 3 == 1) {
                size *= 0.4;
            } else {
                size *= 0.5;
            }
            ++i;
        }
        // If the start grid size is too small
        while (gridRes < minWorldSpacing) {
            if (i % 3 == 0) {
                gridRes *= 2;
            } else if (i % 3 == 1) {
                gridRes *= 2.5;
            } else {
                gridRes *= 2;
            }
            ++i;
        }
    }

    @Override
    public List<String> getInfo() {
        return new ArrayList<String>(Arrays.asList("Grid spacing: " + gridRes));
    }

    @Override
    public String[] getHoverInfo(Vec2 worldPos) {
        return null;
    }

    @Override
    public JMenuItem[] getClicMenuItem(Vec2 worldPos) {
        return null;
    }

}