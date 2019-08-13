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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.composite;

import java.util.HashMap;

/**
    A BusComposite represents a class or structure with members.
    The contents map should be filled with the member name and sub-component.
 */
public class BusComposite implements BusComponent {
    private HashMap<String, BusComponent> contents = new HashMap<>();

    public void setComponent(String key, BusComponent comp){
        contents.put(key, comp);
    }
    public BusComponent getComponent(String key){
        return contents.get(key);
    }

    HashMap<String, BusComponent> getContents(){
        return contents;
    }


    public ComponentType getType(){
        return ComponentType.COMPOSITE;
    }
}
