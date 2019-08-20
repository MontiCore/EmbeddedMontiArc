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

import java.time.Instant;

import commons.simulation.DiscreteEvent;

/**
 * Class that holds all relevant information for a network discrete event
 */
public class NetworkDiscreteEvent extends DiscreteEvent {

    /** Time of the event */
    private Instant eventTime;

    /** Type of the event */
    private NetworkDiscreteEventId eventId;

    /** Message of the event */
    private NetworkMessage eventMessage;

    /** Network node related to this event */
    private NetworkNode networkNode;

    /**
     * Constructor for a network discrete event
     * @param eventTime Time of the event measured in nanoseconds
     * @param eventId Id of the event
     * @param networkNode Network node related to this event
     * @param eventMessage Message of the event
     */
    public NetworkDiscreteEvent(Instant eventTime, NetworkDiscreteEventId eventId, NetworkNode networkNode, NetworkMessage eventMessage) {
        this.eventTime = eventTime;
        this.eventId = eventId;
        this.eventMessage = eventMessage;
        this.networkNode = networkNode;
    }

    /**
     * Function that returns the network event id of the event
     *
     * @return Network event id of the event
     */
    public NetworkDiscreteEventId getNetworkEventId() {
        return eventId;
    }

    /**
     * Function that returns the message of the event
     *
     * @return Message of the event
     */
    public NetworkMessage getEventMessage() {
        return eventMessage;
    }

    /**
     * Function that returns the network node of the event
     *
     * @return Network node of the event
     */
    public NetworkNode getNetworkNode() {
        return networkNode;
    }

    /**
     * Function that returns the time of the event
     *
     * @return Time of the event
     */
    @Override
    public Instant getEventTime() {
        return eventTime;
    }

    /**
     * Function that returns a numeric identifier for the event
     *
     * @return Numeric identifier for the event
     */
    @Override
    public int getEventId() {
        return eventId.ordinal();
    }

    /**
     * Improved toString() method to get more information
     *
     * @return String of information about object
     */
    @Override
    public String toString() {
        return "NetworkDiscreteEvent{" +
                "eventTime=" + eventTime +
                ", eventId=" + eventId +
                ", eventMessage=" + eventMessage +
                ", networkNode=" + networkNode +
                '}';
    }
}