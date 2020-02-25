/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

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
