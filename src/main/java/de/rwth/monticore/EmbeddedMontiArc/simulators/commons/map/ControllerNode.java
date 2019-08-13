/**
 *
 *  ******************************************************************************
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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;
import java.util.HashMap;

/**
 * Created by lukas on 31.01.17.
 */
public class ControllerNode implements IControllerNode{

    private static int counter = 0;

    private Point3D p;

    private long id;
    private long osmId;

    private static HashMap<Point3D, Long> ids = new HashMap<>();

    public ControllerNode(Point3D p, long osmId) {
        this.p = p;
        if(!ids.containsKey(p)) {
            this.id = ++counter;
            ids.put(p, this.id);
        } else {
            this.id = ids.get(p);
        }
        this.osmId = osmId;
    }

    @Override
    public Point3D getPoint() {
        return this.p;
    }

    @Override
    public long getId() {
        return this.id;
    }

    @Override
    public long getOsmId() {
        return this.osmId;
    }

    public String toString() {
        return p.toString();
    }

    public static int getCounter() {
        return counter;
    }
}
