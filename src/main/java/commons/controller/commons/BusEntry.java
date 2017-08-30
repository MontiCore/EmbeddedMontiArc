package commons.controller.commons;

/**
 * This enum is used to administrate the entries of the Bus.
 *
 * Created by Christoph Grüne on 13.01.2017.
 * @author Christoph Grüne
 */
public enum BusEntry {

    //***************************************************************************
    // Navigation                                                               *
    //***************************************************************************
    NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE("NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE"),
    //***************************************************************************
    // Constants                                                                *
    //***************************************************************************
    CONSTANT_MAXIMUM_TOTAL_VELOCITY("CONSTANT_MAXIMUM_TOTAL_VELOCITY"),
    CONSTANT_NUMBER_OF_GEARS("CONSTANT_NUMBER_OF_GEARS"),
    CONSTANT_WHEELBASE("CONSTANT_WHEELBASE"),
    CONSTANT_MAP_ADJACENCY_LIST("CONSTANT_MAP_ADJACENCY_LIST"),
    CONSTANT_MOTOR_MAX_ACCELERATION("CONSTANT_MOTOR_MAX_ACCELERATION"),
    CONSTANT_MOTOR_MIN_ACCELERATION("CONSTANT_MOTOR_MIN_ACCELERATION"),
    CONSTANT_BRAKES_MAX_ACCELERATION("CONSTANT_BRAKES_MAX_ACCELERATION"),
    CONSTANT_BRAKES_MIN_ACCELERATION("CONSTANT_BRAKES_MIN_ACCELERATION"),
    CONSTANT_STEERING_MAX_ANGLE("CONSTANT_STEERING_MAX_ANGLE"),
    CONSTANT_STEERING_MIN_ANGLE("CONSTANT_STEERING_MIN_ANGLE"),
    CONSTANT_TRAJECTORY_ERROR("CONSTANT_TRAJECTORY_ERROR"),
    //***************************************************************************
    // Sensor Entries                                                           *
    //***************************************************************************
    SENSOR_VELOCITY("SENSOR_VELOCITY"),
    SENSOR_STEERING("SENSOR_STEERING"),
    SENSOR_DISTANCE_TO_RIGHT("SENSOR_DISTANCE_TO_RIGHT"),
    SENSOR_DISTANCE_TO_LEFT("SENSOR_DISTANCE_TO_LEFT"),
    SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR("SENSOR_LEFT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR"),
    SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR("SENSOR_RIGHT_FRONT_WHEEL_DISTANCE_TO_STREET_SENSOR"),
    SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR("SENSOR_LEFT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR"),
    SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR("SENSOR_RIGHT_BACK_WHEEL_DISTANCE_TO_STREET_SENSOR"),
    SENSOR_GPS_COORDINATES("SENSOR_GPS_COORDINATES"),
    SENSOR_CURRENT_SURFACE("SENSOR_CURRENT_SURFACE"),
    SENSOR_WEATHER("SENSOR_WEATHER"),
    SENSOR_CAMERA("SENSOR_CAMERA"),
    SENSOR_COMPASS("SENSOR_COMPASS"),
    SENSOR_OBJECT("SENSOR_OBJECT"),
    SENSOR_VANISHINGPOINT("SENSOR_VANISHING"),
    SENSOR_LANE("SENSOR_LANE"),
    //***************************************************************************
    //*  Actuator Entries                                                       *
    //***************************************************************************
    ACTUATOR_ENGINE("ACTUATOR_ENGINE"),
    ACTUATOR_BRAKE("ACTUATOR_BRAKE"),
    ACTUATOR_STEERING("ACTUATOR_STEERING"),
    ACTUATOR_GEAR("ACTUATOR_GEAR"),
    //***************************************************************************
    //*  Computervision Entries                                                 *
    //***************************************************************************
    COMPUTERVISION_VANISHING_POINT("COMPUTERVISION_VANISHING_POINT"),
    COMPUTERVISION_DETECTED_CARS("COMPUTERVISION_DETECTED_CARS"),
    COMPUTERVISION_TRACKED_CARS("COMPUTERVISION_TRACKED_CARS"),
    COMPUTERVISION_TRACKED_PEDESTRIANS("COMPUTERVISION_DETECTED_PEDESTRIANS"),
    COMPUTERVISION_DETECTED_PEDESTRIANS("COMPUTERVISION_DETECTED_PEDESTRIANS"),
    COMPUTERVISION_DETECTED_LANES("COMPUTERVISION_DETECTED_LANES"),
    COMPUTERVISION_DETECTED_SIGNS("COMPUTERVISION_DETECTED_SIGNS"),

    VEHICLE_MAX_TEMPORARY_ALLOWED_VELOCITY("VEHICLE_MAX_TEMPORARY_ALLOWED_VELOCITY"),
    SIMULATION_DELTA_TIME("SIMULATION_DELTA_TIME"),
    ENVIRONMENT("ENVIRONMENT");

    /**
     * this variable contains the name of the entry
     */
    private final String name;

    /**
     * constructor for a bus entry
     *
     * @param name the name of the entry
     */
    private BusEntry(String name) {
        this.name = name;
    }

    /**
     * Getter of the name attribute
     *
     * @return the name of the entry
     */
    public String toString() {
        return this.name;
    }
}
