package de.rwth.montisim.simulation.simulator.visualization.map;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;
import java.awt.Graphics2D;

import javax.swing.JMenuItem;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.simulator.visualization.ui.Renderer;
import de.rwth.montisim.simulation.eecomponents.navigation.Pathfinding;
import de.rwth.montisim.simulation.environment.world.World;

public class PathfinderRenderer extends Renderer {
    public static final Color PATH_COLOR = new Color(255, 100, 100, 100);
    public static final BasicStroke PATH_STROKE = new BasicStroke(4);

    Vec2 startPosition = new Vec2(0.5,0);
    Vec2 targetPosition = new Vec2(3,3.5);

    Pathfinding pf;
    Vec2 res[] = null;

    final List<Polyline> lines = new ArrayList<>();

    public PathfinderRenderer(World world) {
        pf = new Pathfinding(world);
        updatePath();
    }

    void updatePath() {
        if (startPosition.x == Double.POSITIVE_INFINITY || targetPosition.x == Double.POSITIVE_INFINITY)
            return;
        try {
            lines.clear();
            res = pf.findShortestPath(startPosition, targetPosition);
            if (res.length == 0) {
                return;
            }

            Polyline path = new Polyline(res.length, PATH_COLOR, PATH_STROKE);
            lines.add(path);

            int i = 0;
            for (Vec2 ip : res) {
                path.points[i] = new Vec3(ip.x, ip.y, 1); // Transform to extended 2D coordinates
                ++i;
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
    public String[] getInfo() {
        return null;
    }

    @Override
    public String[] getHoverInfo(Vec2 worldPos) {
        return null;
    }

    @Override
    public JMenuItem[] getClicMenuItem(Vec2 worldPos) {
        return new JMenuItem[] {
            new SetTargetMenuItem("Set Start position", worldPos, startPosition),
            new SetTargetMenuItem("Set Target position", worldPos, targetPosition)
        };
    }
    
    public class SetTargetMenuItem extends JMenuItem implements ActionListener {
        private static final long serialVersionUID = 170035139576795954L;
    
        final Vec2 value;
        final Vec2 target;
    
        // Preview: wether the text to be copied is shown in parenthesis after the button message
        public SetTargetMenuItem(String msg, Vec2 value, Vec2 target) {
            super(msg);
            addActionListener(this);
            this.value = value;
            this.target = target;
        }
    
        @Override
        public void actionPerformed(ActionEvent e) {
            target.set(value);
            updatePath();
        }
    }
}