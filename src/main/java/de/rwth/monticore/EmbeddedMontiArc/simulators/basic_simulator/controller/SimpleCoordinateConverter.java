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
package de.rwth.monticore.EmbeddedMontiArc.simulators.basic_simulator.controller;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;


/*
    https://en.wikipedia.org/wiki/Equirectangular_projection
    https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
*/
public class SimpleCoordinateConverter {
    private static final double EARTH_RADIUS = 12742000*0.5; //In meters
    private static final double DEG_TO_RAD = 2.0 * Math.PI / 360.0;
    private static final double RAD_TO_DEG = 1 / DEG_TO_RAD;
    private static final double DEG_TO_METERS = EARTH_RADIUS * DEG_TO_RAD;
    private static final double METERS_TO_DEG = 1 / DEG_TO_METERS;

    private double lon_ref;
    private double lat_ref;
    private double lon_factor;
    private double inv_lon_factor;

    public SimpleCoordinateConverter(Point2D ref){
        lon_ref = ref.getY();
        lat_ref = ref.getX();
        lon_factor = Math.cos(lat_ref*DEG_TO_RAD) * DEG_TO_METERS;
        inv_lon_factor = 1 / lon_factor;
    }

    public Point2D latlonToMeters(Point2D latlon){
        return new Point2D( (latlon.getY()-lon_ref) *lon_factor, (latlon.getX()-lat_ref) * DEG_TO_METERS);
        //return new Point2D( (latlon.getY()-lon_ref) *DEG_TO_METERS, (latlon.getX()-lat_ref) * DEG_TO_METERS);
    }

    public Point2D metersToLatlon(Point2D meters){
        return new Point2D(meters.getY()*METERS_TO_DEG + lat_ref, meters.getX()*inv_lon_factor + lon_ref);
    }
}