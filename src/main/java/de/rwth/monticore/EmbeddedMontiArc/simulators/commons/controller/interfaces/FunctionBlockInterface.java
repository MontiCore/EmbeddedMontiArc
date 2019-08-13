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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces;

import java.util.Map;

/**
 * This interface provides the basic structures of a function block
 *
 * Created by Christoph Grüne on 09.12.2016.
 * @author Christoph Grüne
 */
public interface FunctionBlockInterface {

    /***************************************************************************
     *  Main function for this function block                                  *
     ***************************************************************************/
    /**
     * main method
     */
    public abstract void execute(double timeDelta);

    /***************************************************************************
     *  Getter and Setter to emulate the Input and Output of a function block  *
     ***************************************************************************/
    /**
     * set connectionMap from extern
     *
     * @param inputs map that contains all connectionMap with in getImportNames specified keys
     */
    public void setInputs(Map<String, Object> inputs);

    /**
     * output method
     *
     * @return all outputs in a map
     */
    public abstract Map<String, Object> getOutputs();

    /**
     * returns all import names for maps
     *
     * @return all import names
     */
    public abstract String[] getImportNames();
}


