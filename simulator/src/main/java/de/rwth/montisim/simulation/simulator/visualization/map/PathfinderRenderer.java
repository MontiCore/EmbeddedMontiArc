/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.map;

import java.awt.*;
import java.awt.event.*;
import java.util.*;
import java.util.List;

import javax.swing.JMenuItem;

import de.rwth.montisim.commons.map.Path;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.simulator.visualization.ui.Renderer;

public class PathfinderRenderer extends Renderer {
    public static final Color PATH_COLOR = new Color(255, 100, 100, 100);
    public static final BasicStroke PATH_STROKE = new BasicStroke(4);

    Optional<Vec2> startPosition = Optional.empty();
    Optional<Vec2> targetPosition = Optional.empty();

    Pathfinding pathfinding;
    Path res = null;

    final List<Polyline> lines = new ArrayList<>();

    public PathfinderRenderer(Pathfinding pathfinding) {
        load(pathfinding);
    }

    public void load(Pathfinding pathfinding) {
        this.pathfinding = pathfinding;
        updatePath();
    }

    void updatePath() {
        lines.clear();
        if (!(startPosition.isPresent() && targetPosition.isPresent()))
            return;
        try {
            res = pathfinding.findShortestPath(startPosition.get(), targetPosition.get());
            final int length = res.getLength();
            if (length == 0) {
                return;
            }

            Polyline path = new Polyline(length, PATH_COLOR, PATH_STROKE);
            lines.add(path);

            for (int i = 0; i < length; ++i) {
                path.points[i] = new Vec3(res.trajectoryX[i], res.trajectoryY[i], 1); // Transform to extended 2D coordinates
            }

            dirty = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void draw(Graphics2D g) {
        drawLines(g, lines);
    }

    @Override
    public void computeGeometry(Mat3 viewMatrix) {
        computeLineGeometry(viewMatrix, lines);
    }

    @Override
    public List<String> getInfo() {
        return null;
    }

    @Override
    public String[] getHoverInfo(Vec2 worldPos) {
        return null;
    }

    @Override
    public JMenuItem[] getClicMenuItem(Vec2 worldPos) {
        return new JMenuItem[]{
                new SetTargetMenuItem("Set Start position", worldPos, true),
                new SetTargetMenuItem("Set Target position", worldPos, false),
                new ClearTargetsMenuItem("Clear Path")
        };
    }

    public class SetTargetMenuItem extends JMenuItem implements ActionListener {
        private static final long serialVersionUID = 170035139576795954L;

        final Vec2 value;
        final boolean changeStart;

        // Preview: wether the text to be copied is shown in parenthesis after the button message
        public SetTargetMenuItem(String msg, Vec2 value, boolean changeStart) {
            super(msg);
            addActionListener(this);
            this.value = value;
            this.changeStart = changeStart;
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            if (changeStart) {
                startPosition = Optional.of(value);
            } else {
                targetPosition = Optional.of(value);
            }
            updatePath();
        }
    }

    public class ClearTargetsMenuItem extends JMenuItem implements ActionListener {
        private static final long serialVersionUID = 5044641911354759117L;

        // Preview: wether the text to be copied is shown in parenthesis after the
        // button message
        public ClearTargetsMenuItem(String msg) {
            super(msg);
            addActionListener(this);
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            startPosition = Optional.empty();
            targetPosition = Optional.empty();
            updatePath();
        }
    }
}