package commons.simulation;

/**
 * Enum for the types of physical objects
 */
public enum PhysicalObjectType {
    /**
     * Type for cars, might be extended to support different models
     * PHYSICAL_OBJECT_TYPE_CAR_DEFAULT used as default
     */
    PHYSICAL_OBJECT_TYPE_CAR_DEFAULT,
    PHYSICAL_OBJECT_TYPE_CAR_1,
    PHYSICAL_OBJECT_TYPE_CAR_2,
    PHYSICAL_OBJECT_TYPE_CAR_3,
    PHYSICAL_OBJECT_TYPE_CAR_4,
    PHYSICAL_OBJECT_TYPE_CAR_5,

    /**
     * Type for pedestrians
     */
    PHYSICAL_OBJECT_TYPE_PEDESTRIAN,

    /**
     * Type for trees
     */
    PHYSICAL_OBJECT_TYPE_TREE,

    
    PHYSICAL_OBJECT_TYPE_STREETLANTERN,

    PHYSICAL_OBJECT_TYPE_ROADWORKSIGN,

    PHYSICAL_OBJECT_TYPE_HOUSE,
    
    
    
    /**
     * Type for network cell base station
     */
    
    PHYSICAL_OBJECT_TYPE_NETWORK_CELL_BASE_STATION,
}
