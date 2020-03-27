/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.map.visualization;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.Locale;
import java.util.Optional;

import java.awt.datatransfer.StringSelection;
import java.awt.Toolkit;
import java.awt.datatransfer.Clipboard;

import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.event.MouseInputListener;

import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;

/**
 * This class allows to see and navigate a [Map]. Use the scroll wheel to zoom
 * in and out. Hold the middle mouse button to move the map around.
 */
public class MapViewer extends JPanel implements MouseInputListener, MouseWheelListener {
    private static final long serialVersionUID = 4687042051904715366L;
    private static final double SCROLL_FACTOR = 1.2;
    public static final Color TEXT_COLOR = new Color(50, 50, 50);
    private static final DecimalFormat format = new DecimalFormat("##0.00",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));
    private static final DecimalFormat geoFormat = new DecimalFormat("##0.0000000",
            DecimalFormatSymbols.getInstance(Locale.ENGLISH));

    private boolean antialiasing = false;
    private Vec2 center = new Vec2();
    // Scale of 1: 1 Meter = 1 Pixel
    // Scale of 2: 1 Meter = 2 Pixels
    private double scale = 1;
    private int scrollCount = 0;
    private Vec2 screenCenter = new Vec2();
    private boolean dirty = true;
    private Mat3 viewMatrix = new Mat3();
    private Mat3 invViewMatrix = new Mat3();
    private boolean trackingMouse = false;
    private final Vec2 mousePos = new Vec2();
    private boolean hover = false;

    private Optional<MapRenderer> map = Optional.empty();

    public MapViewer() {
        this.addMouseListener(this);
        this.addMouseWheelListener(this);
        this.addMouseMotionListener(this);
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        if (antialiasing) {
            RenderingHints rh = new RenderingHints(RenderingHints.KEY_TEXT_ANTIALIASING,
                    RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
            rh.put(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g2.setRenderingHints(rh);
        }

        // Check panel size
        Dimension d = this.getSize();
        Vec2 size = new Vec2(d.getWidth(), d.getHeight());
        Vec2 zoneCenter = size.multiply(0.5);
        if (!zoneCenter.equals(screenCenter)) {
            dirty = true;
            screenCenter = zoneCenter;
        }

        if (map.isPresent()) {
            MapRenderer mr = map.get();
            if (dirty) {
                viewMatrix = computeViewMatrix();
                invViewMatrix = computeInvViewMatrix();
                mr.computeGeometry(viewMatrix);
                dirty = false;
            }
            mr.draw(g);
        }

        renderInfoText(g);
    }

    public void renderInfoText(Graphics g){
        int offset = g.getFont().getSize() + 3;
        g.setColor(TEXT_COLOR);
        if (map.isPresent()){
            MapRenderer mr = map.get();

            g.drawString("Mouse wheel to zoom.", 5, offset);
            g.drawString("Hold middle mouse button to move.", 5, offset * 2);
            g.drawString("Right clic to copy coordinates.", 5, offset * 3);
    
            if (hover) {
                Vec2 wp = getWorldPos(mousePos);
                String info = "x: " + format.format(wp.x) + " y: " + format.format(wp.y);
                if (mr.map.converter.isPresent()) {
                    Coordinates coords = mr.map.converter.get().metersToCoords(wp);
                    info += " (lon: " + geoFormat.format(coords.lon) + " lat: " + geoFormat.format(coords.lat) + ")";
                }
                g.drawString(info, 5, offset * 4);
            }
        } else {
            g.drawString("No Map", 5, offset);
        }
    }

    public void setMap(MapRenderer mr) {
        this.map = Optional.of(mr);
        this.dirty = true;
    }

    public void translate(Vec2 mouseMotion) {
        // /!\ mouseMotion is in "Screen Space".
        double scaleInv = 1 / scale;
        center = new Vec2(center.x - mouseMotion.x * scaleInv, center.y + mouseMotion.y * scaleInv);
        dirty = true;
    }

    public void zoom(int steps, Vec2 mousePos) {
        scrollCount -= steps;
        double newScale = Math.pow(SCROLL_FACTOR, scrollCount);
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

    public void setAntialiasing(boolean aa) {
        this.antialiasing = aa;
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
            if (map.isPresent() && map.get().map.converter.isPresent()){
                Coordinates coords = map.get().map.converter.get().metersToCoords(getWorldPos(new Vec2(e.getX(), e.getY())));
                String info = geoFormat.format(coords.lon) + ", " + geoFormat.format(coords.lat);
                new CopyMenu(info).show(e.getComponent(), e.getX(), e.getY());
            }
        }
        if (e.getButton() == MouseEvent.BUTTON2) {
            trackingMouse = false;
        }
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

    class CopyMenu extends JPopupMenu implements ActionListener {
        private static final long serialVersionUID = -8970507839473030743L;
        JMenuItem copyItem;
        String coords;
        public CopyMenu(String coords) {
            this.coords = coords;
            copyItem = new JMenuItem("Copy Coordinates");
            add(copyItem);
            copyItem.addActionListener(this);
        }

        @Override
        public void actionPerformed(ActionEvent e) {
            StringSelection stringSelection = new StringSelection(coords);
            Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
            clipboard.setContents(stringSelection, null);
        }
    }
}