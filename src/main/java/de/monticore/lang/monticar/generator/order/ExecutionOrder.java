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
package de.monticore.lang.monticar.generator.order;

/**
 * Based on the ExecutionOrder file of the montiarc-executionorder project.
 * It has to be included here, as another ExpandedComponentInstanceSymbol(from embedded-montiarc) is used.
 * Which does not extends the montiarc-executionorder ExpandedComponentInstanceSymbol.
 * Also using this as a possibility to provide an implementation for an embedded-montiarc-math generator
 * that can be reused while only the target language implementation has to change.
 *
 */
public interface ExecutionOrder extends Comparable {

    String INTERFACE_NAME = "ExecutionOrder";

    String toString();

    int getS();

    int getB();

    boolean equals(Object obj);

    int hashCode();

    int compareTo(Object obj);

}
