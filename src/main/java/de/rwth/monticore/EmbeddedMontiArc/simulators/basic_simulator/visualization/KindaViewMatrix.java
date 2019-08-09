/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
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