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
package de.monticore.lang.math.math._matrixprops;


/**
 * Created by Philipp Goerick on 02.09.2017.
 *
 * Computes matrix properties of a matrix expression
 */

public enum MatrixProperties {
    Diag, Herm, Indef, NegDef, NegSemDef, Norm, PosDef, PosSemDef, SkewHerm, Square, Invertible, Positive, Negative;

    /**
     * convert the enum type based properties to a string
     *
     * @return in String {@link String} formatted matrix properties
     */
    @Override
    public String toString() {
        switch(this) {
            case Diag:{
                return "diag";
            }
            case Herm:{
                return "herm";
            }
            case Norm:{
                return "norm";
            }
            case Indef:{
                return "indef";
            }
            case NegDef:{
                return "nd";
            }
            case PosDef:{
                return "pd";
            }
            case Square:{
                return "square";
            }
            case SkewHerm:{
                return "skewHerm";
            }
            case NegSemDef:{
                return "nsd";
            }
            case PosSemDef:{
                return "psd";
            }
            case Invertible:{
                return "inv";
            }
            case Positive:{
                return "pos";
            }
            case Negative:{
                return "neg";
            }
            default:{
                return "noProps";
            }
        }
    }
}
