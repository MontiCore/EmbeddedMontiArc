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
package de.monticore.lang.monticar.generator.optimization;

/**
 * @author Sascha Schneiders
 */
public class ResultOrder {
    public int matrixIndexLeft;
    public int matrixIndexRight;
    public boolean isResultLeft;
    public boolean isResultRight;

    public ResultOrder(int matrixIndexLeft, boolean isResultLeft, int matrixIndexRight, boolean isResultRight) {
        this.matrixIndexLeft = matrixIndexLeft;
        this.matrixIndexRight = matrixIndexRight;
        this.isResultLeft = isResultLeft;
        this.isResultRight = isResultRight;
    }
}
