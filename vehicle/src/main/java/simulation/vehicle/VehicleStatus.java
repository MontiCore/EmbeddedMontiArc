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
package simulation.vehicle;

/**
 * State of vehicle parts
 */
public enum VehicleStatus {
    /** Status is OK. No problems sensed. */
    VEHICLE_STATUS_OK(1000),

    /** Status is OK, but component should be inspected in workshop */
    VEHICLE_STATUS_SERVICE_REQUIRED(2000),

    /** Component damaged, but still working */
    VEHICLE_STATUS_DAMAGED(3000),

    /** Failure of the component imminent */
    VEHICLE_STATUS_CRITICAL(4000),

    /** Component is broken */
    VEHICLE_STATUS_FAILURE(5000);

    /** Severeness of the status. Higher means more critical. */
    private Integer severeness;

    /**
     * Constructor for a new status
     * @param severeness Severeness of the status. Higher means more critical.
     */
    VehicleStatus(int severeness) {
        this.severeness = severeness;
    }

    /**
     * Compares two states
     * @param other Level to compare caller to
     * @return True iff the caller has a higher or equal priority than other
     */
    public boolean isWorseThanOrEqual(VehicleStatus other) {
        return this.severeness >= other.severeness;
    }
}