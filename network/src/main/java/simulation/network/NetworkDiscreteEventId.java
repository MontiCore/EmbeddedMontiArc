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
 * Enum that defines all identifiers for network events
 * Task specific identifiers may be included as well
 */
public enum NetworkDiscreteEventId {
    NETWORK_EVENT_ID_NONE,

    NETWORK_EVENT_ID_RANDOM_START_INITIALIZE,
    NETWORK_EVENT_ID_SELF_PERIODIC,

    NETWORK_EVENT_ID_APP_RECEIVE,
    NETWORK_EVENT_ID_APP_UPDATE,
    NETWORK_EVENT_ID_APP_SEND,

    NETWORK_EVENT_ID_TRANSPORT_RECEIVE,
    NETWORK_EVENT_ID_TRANSPORT_SEND,

    NETWORK_EVENT_ID_NET_RECEIVE,
    NETWORK_EVENT_ID_NET_SEND,
    NETWORK_EVENT_ID_NET_MULTICAST,

    NETWORK_EVENT_ID_LINK_RECEIVE,
    NETWORK_EVENT_ID_LINK_SEND,
    NETWORK_EVENT_ID_LINK_CHECK_CHANNEL_STATUS,
    NETWORK_EVENT_ID_LINK_FORWARD_TO_PHY,
    NETWORK_EVENT_ID_LINK_WAIT_FOR_SENDING,

    NETWORK_EVENT_ID_PHY_RECEIVE_MESSAGE_START,
    NETWORK_EVENT_ID_PHY_RECEIVE_MESSAGE_END,
    NETWORK_EVENT_ID_PHY_RECEIVE_INTERFERENCE_START,
    NETWORK_EVENT_ID_PHY_RECEIVE_INTERFERENCE_END,
    NETWORK_EVENT_ID_PHY_RECEIVE_INTERRUPTION_DETECTED,
    NETWORK_EVENT_ID_PHY_SEND_START,
    NETWORK_EVENT_ID_PHY_SEND_END,
}