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
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

/**
 * Enum for the types of physical objects
 */
public enum PhysicalObjectType {
    /**
     * Type for cars, might be extended to support different models
     * PHYSICAL_OBJECT_TYPE_CAR_DEFAULT used as default
     */
    PHYSICAL_OBJECT_TYPE_CAR,

    /**
     * Type for pedestrians
     */
    PHYSICAL_OBJECT_TYPE_PEDESTRIAN,

    /**
     * Type for trees
     */
    PHYSICAL_OBJECT_TYPE_TREE,

    
    PHYSICAL_OBJECT_TYPE_STREET_LANTERN,

    PHYSICAL_OBJECT_TYPE_ROAD_WORK_SIGN,

    PHYSICAL_OBJECT_TYPE_HOUSE,
    
    
    
    /**
     * Type for network cell base station
     */
    
    PHYSICAL_OBJECT_TYPE_NETWORK_CELL_BASE_STATION,
}
