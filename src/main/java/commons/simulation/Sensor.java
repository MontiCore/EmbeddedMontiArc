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
package commons.simulation;

import commons.controller.commons.BusEntry;

import java.time.Instant;

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
    void update(Instant actualTime);

    /**
     *function that returns the length of the data
     * @return individual lenght of the data of the sensor in bytes
     */
    int getDataLength();

}
