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
package simulation.environment.pedestrians;

import commons.utils.Point3D;

/**
 * Created by lukas on 07.02.17.
 *
 * A container for the parameters needed to compute the movement of a pedestrian
 */
public class PedestrianStreetParameters {
    private boolean isCrossing;
    private boolean direction;
    private boolean leftPavement;
    private Point3D position;

    public PedestrianStreetParameters(boolean isCrossing, Point3D position, boolean direction, boolean leftPavement) {
        this.isCrossing = isCrossing;
        this.position = position;
        this.direction = direction;
        this.leftPavement = leftPavement;
    }

    /**
     * @return the position of the pedestrian
     */
    public Point3D getPosition() {
        return position;
    }

    /**
     * @return true iff the pedestrian crosses the street
     */
    public boolean isCrossing() {
        return isCrossing;
    }

    /**
     * @return true if the pedestrian walks from p3D1 to p3D2. false if the pedestrian walks from p3D2 to p3D1. might have undefined behaviour
     * while the pedestrian crosses the street
     */
    public boolean isDirection() {
        return direction;
    }

    /**
     * @return true iff the pedestrian walks on the left pavement
     */
    public boolean isLeftPavement() {
        return this.leftPavement;
    }
}