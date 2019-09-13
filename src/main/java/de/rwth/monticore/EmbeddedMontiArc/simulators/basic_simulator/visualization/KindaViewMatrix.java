/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.visualization;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Pair;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;
import java.awt.Dimension;

public class KindaViewMatrix {
    int corner_x;
    int corner_y;
    Point2D min_coords;
    double ratio;
    public KindaViewMatrix(Dimension d, Point2D coord_span, Point2D min_coords){
        this.min_coords = min_coords;
        int area_width = (int) d.getWidth()-1;
        int area_height = (int) d.getHeight()-1;
        double x_ratio = area_width / coord_span.getX();
        double y_ratio = area_height / coord_span.getY();
        ratio = Math.min(x_ratio, y_ratio) * 0.95;
        Point2D offset = coord_span.multiply(0.5*ratio);
        corner_x = area_width/2 - (int)offset.getX();
        corner_y = area_height/2 + (int)offset.getY();
    }

    public Pair<Integer,Integer> get_screen_coords(Point2D coords){
        Point2D rel = coords.subtract(min_coords).multiply(ratio);
        return new Pair<>(corner_x + (int) rel.getX(), corner_y - (int) rel.getY());
    }
}