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
package simulation.environment.osm;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;

/**
 * Created by lukas on 24.01.17.
 * Specifies the Methods of a Converter from longitude/latitude to metric x and y
 */
public interface MetricConverter {

    /**
     *
     * @param longitude
     * @param latitude
     * @return the metric value according to the given longitude in dependence of the latitude
     */
    public abstract double convertLongToMeters(double longitude, double latitude);

    /**
     * @param latitude
     * @return the metric value according to the given latitude
     */
    public abstract  double convertLatToMeters(double latitude);

    /**
     * @param p
     * @return a point with the according metric values
     */
    public abstract Point3D convertLongLatPoint(Point3D p);
}