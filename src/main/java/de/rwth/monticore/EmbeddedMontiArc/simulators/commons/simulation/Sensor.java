/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;

/**
 * Created by Aklima Zaman on 16-Dec-16. This is an interface for a sensor. It
 * includes all of the mandatory functions of a sensor should be implement.
 */
public interface Sensor {

    /**
     * @return type of the sensor
     */
    BusEntry getType();

    /**
     * @return It will return the value of the sensor.
     */
    Object getValue();

    /**
     * Return an informative string for the name of the type of the value the
     * sensor will return.
     *
     * @return an informative string for the name of the type of the value the
     *         sensor will return.
     */
    String getTypeName();

    /**
     * This method update the value of the sensor.
     */
    void update();

}
