/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator.visualization.ui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.MouseInputListener;

import java.text.*;
import java.util.*;
import java.util.List;

import de.rwth.montisim.commons.utils.*;

/**
 * This class is a JPanel that allows navigating a 2D view.
 * Multiple 2D views can be overlaid. They share a common coordinate system.
 * Use the scroll wheel to zoom in and out.
 * Hold the middle mouse button to move the map around.
 * <p>
 * Possible 2D views:
 * - Map
 * - CarVis
 */
public class Viewer2D extends JPanel implements MouseInputListener, MouseWheelListener {
    private static final long serialVersionUID = 4687042051904715366L;
    // private static final ImageIcon a;
    // static {
    //     java.net.URL imgURL = Control.class.getResource("/images/a.gif");
    //     if (imgURL != null)
    //         a = new ImageIcon(imgURL);
    //     else a = null;
    // }

    private static final DecimalFormat format = new DecimalFormat("##0.00",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));

    private Vec2 center = new Vec2();
    // Scale of 1: 1 Meter = 1 Pixel
    // Scale of 2: 1 Meter = 2 Pixels
    public double scale = 1;
    private int scrollCount = 0;
    private Vec2 screenCenter = new Vec2();
    private boolean dirty = true;
    public Vec2 screen_size = new Vec2();
    public Mat3 viewMatrix = new Mat3();
    private Mat3 invViewMatrix = new Mat3();
    private boolean trackingMouse = false;
    private final Vec2 mousePos = new Vec2();
    private boolean hover = false;
    public boolean copyPos = true;


    private List<Renderer> renderers = new ArrayList<>();
    //private Optional<MapRenderer> map = Optional.empty();

    public Viewer2D() {
        this.addMouseListener(this);
        this.addMouseWheelListener(this);
        this.addMouseMotionListener(this);
        addRenderer(new GridRenderer(this));

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

        // Check panel size
        Dimension d = this.getSize();
        screen_size.x = d.getWidth();
        screen_size.y = d.getHeight();
        Vec2 zoneCenter = screen_size.multiply(0.5);
        if (!zoneCenter.equals(screenCenter)) {
            dirty = true;
            screenCenter = zoneCenter;
        }

        if (dirty) {
            viewMatrix = computeViewMatrix();
            invViewMatrix = computeInvViewMatrix();
        }
        //Iterator<Renderer> iter = renderers.iterator();
        List<Renderer> copy = new ArrayList<>(renderers);
        for (Renderer r : copy) {
            //while(iter.hasNext()){
            if (renderers.contains(r)) {
                if (dirty || r.dirty) {
                    r.computeGeometry(viewMatrix);
                    r.dirty = false;
                }
                r.draw(g2);
            } else {
                return;
            }
        }

        if (dirty) dirty = false;

        renderInfoText(g);

        // if (a != null){
        //     a.paintIcon(this, g, 100, 100);
        // }
    }

    private void renderInfoText(Graphics g) {
        // Get Info content
        List<String> lines = new ArrayList<>();

        lines.add("Mouse wheel to zoom.");
        lines.add("Hold middle mouse button to move.");
        lines.add("Right clic to copy coordinates.");
        lines.add(" ");

        if (UIInfo.drawPlannedPath) lines.add("RED LINE: Planned Path for the Vehicle.");
        if (UIInfo.drawPlannedTrajectory) lines.add("GREEN LINE: Trajectory input to the autopilot.");
        if (UIInfo.drawPlannedPath || UIInfo.drawPlannedTrajectory) lines.add(" ");

        if (UIInfo.drawActuators) {
            lines.add("GREEN CIRCLE on the car: GAS");
            lines.add("RED CIRCLE on the car: BRAKES");
            lines.add(" ");
        }

        for (Renderer r : renderers) {
            List<String> l = r.getInfo();
            if (l != null) {
                lines.add(" ");
                for (String s : l) lines.add(s);
            }
        }

        lines.add(" ");
        if (hover) {
            Vec2 wp = getWorldPos(mousePos);
            lines.add("x: " + format.format(wp.x) + " y: " + format.format(wp.y));
            for (Renderer r : renderers) {
                String[] l = r.getHoverInfo(wp);
                if (l != null) for (String s : l) lines.add(s);
            }
        }

        // Measure size

        FontMetrics metrics = g.getFontMetrics(g.getFont());
        int lineCount = 0;
        int maxWidth = 0;
        for (String s : lines) {
            if (s.length() > 0) {
                lineCount++;
                int width = metrics.stringWidth(s);
                if (width > maxWidth) maxWidth = width;
            }
        }

        // Render
        final int offset = g.getFont().getSize() + UIInfo.LINE_SPACE;
        g.setColor(UIInfo.PANEL_COLOR);
        g.fillRect(0, 0, maxWidth + (2 * UIInfo.MARGIN), lineCount * offset + UIInfo.MARGIN);
        g.setColor(UIInfo.TEXT_COLOR);

        int index = 1;
        for (String s : lines) {
            if (s.length() == 0) continue;
            g.drawString(s, UIInfo.MARGIN, offset * index);
            index++;
        }
    }


    public void addRenderer(Renderer r) {
        this.renderers.add(r);
        r.dirty = true;
    }

    public void clearRenderers() {
        Renderer grid = renderers.get(0);
        renderers.clear();
        renderers.add(grid);
    }

    public void translate(Vec2 mouseMotion) {
        // /!\ mouseMotion is in "Screen Space".
        double scaleInv = 1 / scale;
        center = new Vec2(center.x - mouseMotion.x * scaleInv, center.y + mouseMotion.y * scaleInv);
        dirty = true;
    }

    public void zoom(int steps, Vec2 mousePos) {
        scrollCount -= steps;
        double newScale = Math.pow(UIInfo.SCROLL_FACTOR, scrollCount);
        /*
         * Correct the "center" so that the zoom happens around the mouse => Comes from
         * solving: V*p=V'*p a.k.a.
         * "The point at which is pointed should not move after scaling." where V & V'
         * are the old and new view matrices and p is the mouse position in world space.
         * It is solved manually by using the details of how the view matrix is
         * constructed: V = C*S*T where C is the "Screen Center" translation, S is the
         * scaling & T is the world translation.
         */
        double scaleRatio = scale / newScale;
        Vec2 worldPos = getWorldPos(mousePos);
        Vec2 relPos = worldPos.subtract(center);
        center = new Vec2(-(scaleRatio * relPos.x - worldPos.x), -(scaleRatio * relPos.y - worldPos.y));
        scale = newScale;
        dirty = true;
    }

    /**
     * Sets the number of pixels per meter.
     */
    public void setZoom(double newScale) {
        // Get nearest scroll count
        scrollCount = (int) Math.round(Math.log(newScale) / Math.log(UIInfo.SCROLL_FACTOR));
        scale = Math.pow(UIInfo.SCROLL_FACTOR, scrollCount);
        dirty = true;
    }

    /**
     * Sets the World coordinates that should be at the center of the screen.
     */
    public void setCenter(Vec2 center) {
        this.center.set(center);
        dirty = true;
    }

    public Vec2 getWorldPos(Vec2 screenPos) {
        Vec3 res = invViewMatrix.multiply(new Vec3(screenPos.x, screenPos.y, 1));
        return new Vec2(res.x, res.y);
    }

    private Mat3 computeViewMatrix() {
        Mat3 mapPos = Mat3.translationMatrix(center.multiply(-1));
        Mat3 mapScale = Mat3.scaleMatrix(new Vec2(scale, -scale));
        Mat3 screenPos = Mat3.translationMatrix(screenCenter);
        return screenPos.multiply(mapScale).multiply(mapPos);
    }

    private Mat3 computeInvViewMatrix() {
        Mat3 mapPosInv = Mat3.translationMatrix(center);
        double invScale = 1 / scale;
        Mat3 mapScaleInv = Mat3.scaleMatrix(new Vec2(invScale, -invScale));
        Mat3 screenPosInv = Mat3.translationMatrix(screenCenter.multiply(-1));
        return mapPosInv.multiply(mapScaleInv).multiply(screenPosInv);
    }

    @Override
    public void mouseClicked(MouseEvent e) {
    }

    @Override
    public void mousePressed(MouseEvent e) {
        // if (e.isPopupTrigger()){
        // new CopyMenu().show(e.getComponent(), e.getXOnScreen(), e.getYOnScreen());
        // }
        if (e.getButton() == MouseEvent.BUTTON2) {
            trackingMouse = true;
            mousePos.x = e.getX();
            mousePos.y = e.getY();
        }
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        if (e.isPopupTrigger()) {
            Vec2 wp = getWorldPos(new Vec2(e.getX(), e.getY()));
            List<JMenuItem> items = new ArrayList<>();

            if (copyPos) {
                String pos = format.format(wp.x) + ", " + format.format(wp.y);
                items.add(new CopyMenuItem("Copy position ", pos, true));
            }
            for (Renderer r : renderers) {
                JMenuItem[] l = r.getClicMenuItem(wp);
                if (l != null) for (JMenuItem s : l) items.add(s);
            }

            if (items.size() > 0) {
                JPopupMenu menu = new JPopupMenu();
                for (JMenuItem i : items) {
                    menu.add(i);
                }
                menu.show(e.getComponent(), e.getX(), e.getY());
            }
        }
        if (e.getButton() == MouseEvent.BUTTON2) {
            trackingMouse = false;
        }
    }

    public void update() {
        repaint();
        revalidate();
    }

    @Override
    public void mouseEntered(MouseEvent e) {
        hover = true;
    }

    @Override
    public void mouseExited(MouseEvent e) {
        hover = false;
        repaint();
    }

    @Override
    public void mouseDragged(MouseEvent e) {
        if (trackingMouse) {
            Vec2 delta = new Vec2(e.getX() - mousePos.x, e.getY() - mousePos.y);
            translate(delta);
        }
        mousePos.x = e.getX();
        mousePos.y = e.getY();
        repaint();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        mousePos.x = e.getX();
        mousePos.y = e.getY();
        repaint();
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        zoom(e.getWheelRotation(), new Vec2(e.getX(), e.getY()));
        repaint();
    }


    public void setDirty() {
        this.dirty = true;
    }

}