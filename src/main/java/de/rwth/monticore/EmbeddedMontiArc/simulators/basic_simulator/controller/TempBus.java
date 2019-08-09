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

import java.util.HashMap;
import java.util.Map;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.interfaces.Bus;

public class TempBus implements Bus {
    private Map<String, Object> busMap= new HashMap<String, Object>();;
    @Override
    public void setData(String key, Object object) {
        busMap.put(key, object);
    }

    @Override
    public void setAllData(Map<String, Object> map) {
        busMap.putAll(map);
    }

    @Override
    public Object getData(String key) {
        return busMap.get(key);
    }

    @Override
    public Map<String, Object> getAllData() {
        return busMap;
    }

    @Override
    public String[] getImportNames() {
        return new String[0];
    }
}
