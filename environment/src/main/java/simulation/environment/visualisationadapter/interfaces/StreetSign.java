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
package simulation.environment.visualisationadapter.interfaces;

import commons.utils.Point3D;

/**
 * Created by lukas on 10.03.17.
 */
public interface StreetSign {

    public abstract SignTypeAndState getSignState();

    public abstract SignTypeAndState getType();

    public abstract long getId();

    public abstract boolean isOne();

    public abstract boolean isTwo();

    public abstract double getX1();

    public abstract double getY1();

    public abstract double getZ1();

    public abstract double getX2();

    public abstract double getY2();

    public abstract double getZ2();

    public abstract void setOne(Point3D p1);

    public abstract void setTwo(Point3D p2);

}