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
package simulation.util;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class OrientedBoundingBox {
    private RealVector position;
    private double halfWidth;
    private double halfLength;
    private double halfHeight;
    private RealVector localAxisX; // Unit vector representing the local x-axis
    private RealVector localAxisY; // Unit vector representing the local y-axis
    private RealVector localAxisZ; // Unit vector representing the local z-axis

    public OrientedBoundingBox(RealVector geometryPosition, double width, double length, double height, RealMatrix rotation) {
        if(geometryPosition.getDimension() != 3){
            throw new IllegalArgumentException("Position " + geometryPosition + " should be three dimensional.");
        }

        if (rotation.getColumnDimension() != 3 || rotation.getRowDimension() != 3) {
            throw new IllegalArgumentException("Rotation " + rotation + " should be a 3x3 matrix");
        }

        // Set position
        position = geometryPosition;

        // Set measurements
        halfWidth = width / 2.0;
        halfLength = length / 2.0;
        halfHeight = height / 2.0;

        // Determine axes
        localAxisX = getAxis(rotation, new double[] {1.0, 0.0, 0.0});
        localAxisY = getAxis(rotation, new double[] {0.0, 1.0, 0.0});
        localAxisZ = getAxis(rotation, new double[] {0.0, 0.0, 1.0});
    }

    public RealVector getPosition() {
        return position;
    }

    public double getHalfWidth() {
        return halfWidth;
    }

    public double getHalfHeight() {
        return halfHeight;
    }

    public double getHalfLength() {
        return halfLength;
    }

    public RealVector getLocalAxisX() {
        return localAxisX;
    }

    public RealVector getLocalAxisY() {
        return localAxisY;
    }

    public RealVector getLocalAxisZ() {
        return localAxisZ;
    }

    private RealVector getAxis(RealMatrix rotation, double[] axisVector) {
        // Rotate the given axis vector and unitize it
        RealVector axis = rotation.operate(new ArrayRealVector(axisVector));
        axis.unitize();

        return axis;
    }
}
