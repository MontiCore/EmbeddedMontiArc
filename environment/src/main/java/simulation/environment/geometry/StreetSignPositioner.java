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
package simulation.environment.geometry;

import commons.utils.Point3D;
import simulation.environment.geometry.osmadapter.GeomStreet;
import simulation.environment.geometry.osmadapter.SplineDeterminator;
import simulation.environment.geometry.splines.Spline;
import simulation.environment.visualisationadapter.implementation.Node2D;
import simulation.environment.visualisationadapter.interfaces.EnvNode;
import simulation.environment.visualisationadapter.interfaces.SignTypeAndState;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by lukas on 20.03.17.
 */
public class StreetSignPositioner {

    public static void positionStreetSigns(List<GeomStreet> streets) {
        for(GeomStreet s : streets) {
            SplineDeterminator deter = s.getDeterminator();
            ArrayList<EnvNode> nodes = new ArrayList<>(s.getObject().getNodes());
            for (int i = 0; i < nodes.size(); i++) {
                Node2D n2 = (Node2D) nodes.get(i);
                if (n2.getStreetSign().getType() != SignTypeAndState.EMPTY_SIGN) {
                    if(i != 0) {
                        Node2D n1 = (Node2D) nodes.get(i - 1);
                        Spline spline = deter.getSplineForPoints(n1.getPoint(), n2.getPoint());
                        if(n1.getPoint().distance(n2.getPoint()) >= deter.getStreet().getStreetWidth().doubleValue() * 0.5) {
                            Point3D difference = spline.getDifference().normalize();
                            Point3D rightBorder = spline.getBorder(false, false);
                            Point3D firstPosition = rightBorder.subtract(difference.multiply(0.5 * deter.getStreet().getStreetWidth().doubleValue()));
                            n2.getStreetSign().setOne(firstPosition);
                        } else {
                            Point3D rightBorder = spline.getBorder(false, true);
                            n2.getStreetSign().setOne(rightBorder);
                        }
                    }

                    if(i != nodes.size() - 1) {
                        Node2D n1 = (Node2D) nodes.get(i + 1);
                        Spline spline = deter.getSplineForPoints(n2.getPoint(), n1.getPoint());
                        if(n1.getPoint().distance(n2.getPoint()) >= deter.getStreet().getStreetWidth().doubleValue() * 0.5) {
                            Point3D difference = spline.getDifference().normalize();
                            Point3D leftBorder = spline.getBorder(true, true);
                            Point3D secondPosition = leftBorder.add(difference.multiply(0.5 * deter.getStreet().getStreetWidth().doubleValue()));
                            n2.getStreetSign().setTwo(secondPosition);
                        } else {
                            Point3D leftBorder = spline.getBorder(true, false);
                            n2.getStreetSign().setTwo(leftBorder);
                        }
                    }
                }
            }
        }

    }
}