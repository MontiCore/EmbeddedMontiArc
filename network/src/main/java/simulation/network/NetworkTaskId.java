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
package simulation.network;

/**
 * Enum that defines all identifiers for network tasks
 */
public enum NetworkTaskId {
    NETWORK_TASK_ID_NONE,

    NETWORK_TASK_ID_APP_BEACON,
    NETWORK_TASK_ID_APP_TRAFFIC_OPTIMIZATION,
    NETWORK_TASK_ID_APP_VELOCITY_CONTROL,
    NETWORK_TASK_ID_APP_MESSAGES_SOFT_STATE,

    NETWORK_TASK_ID_TRANSPORT_SIMPLE,

    NETWORK_TASK_ID_NET_SIMPLE,
    NETWORK_TASK_ID_NET_CELLULAR_MULTICAST,

    NETWORK_TASK_ID_LINK_SIMPLE,
    NETWORK_TASK_ID_LINK_CSMA,
    NETWORK_TASK_ID_LINK_BUFFERED_ROHC,

    NETWORK_TASK_ID_PHY_SIMPLE,
    NETWORK_TASK_ID_PHY_INTERFERENCE,
}