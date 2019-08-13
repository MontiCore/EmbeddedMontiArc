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
 * This is an interface for the DataBus.
 *
 * Created by Christoph Grüne on 30.12.2016.
 * @author Christoph Grüne
 */
public interface Bus {

    /**
     * puts a specific value to a key
     *
     * @param key is the key
     * @param object is the object
     */
    public abstract void setData(String key, Object object);

    /**
     * integrates a existing map into this one
     *
     * @param map is the map
     */
    public void setAllData(Map<String, Object> map);

    /**
     * returns the object of a certain key
     *
     * @param key is the key of the object
     * @return the object with key key
     */
    public Object getData(String key);

    /**
     * returns the connectionMap map
     *
     * @return busMap the Map that contains all information
     */
    public Map<String, Object> getAllData();

    /**
     * returns all import names for maps
     *
     * @return all import names
     */
    public abstract String[] getImportNames();
}
