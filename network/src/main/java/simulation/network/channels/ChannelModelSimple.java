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
package simulation.network.channels;

import simulation.network.*;

import java.time.Instant;
import java.util.LinkedList;
import java.util.List;

/**
 * Class representing a simple channel objects
 */
public class ChannelModelSimple extends NetworkChannelModel {

    /** Channel information: Channel id, center carrier frequency in kHz, channel bandwidth in kHz */
    private static int[][] channelInfo = new int[][] {
        {5890000, 10000},
    };

    /**
     * Function that computes if and how transmissions between network nodes are received
     *
     * @param sender Node that stars the transmission
     * @param otherNode All other nodes that might receive the transmission
     * @param message Message to be sent
     */
    @Override
    public void computeTransmission(NetworkNode sender, NetworkNode otherNode, NetworkMessage message) {
        // Get channel information for sender node
        int channelId = (int)(sender.getPhysicalObject().getId() % 1L);
        int channelCarrierKHz = channelInfo[channelId][0];
        int channelBandwidthKHz = channelInfo[channelId][1];

        // Create events
        Instant transmissionReceiveStart = NetworkUtils.simTimeWithDelay(NetworkUtils.calcPropagationTime(sender, otherNode));
        Instant transmissionReceiveEnd = NetworkUtils.simTimeWithDelay(NetworkUtils.calcPropagationTime(sender, otherNode).plus(NetworkUtils.calcTransmissionTime(message)));
        NetworkDiscreteEvent eventReceiveStart = new NetworkDiscreteEvent(transmissionReceiveStart, NetworkDiscreteEventId.NETWORK_EVENT_ID_PHY_RECEIVE_MESSAGE_START, otherNode, message);
        NetworkDiscreteEvent eventReceiveEnd = new NetworkDiscreteEvent(transmissionReceiveEnd, NetworkDiscreteEventId.NETWORK_EVENT_ID_PHY_RECEIVE_MESSAGE_END, otherNode, message);
        NetworkSimulator.getInstance().scheduleEvent(eventReceiveStart);
        NetworkSimulator.getInstance().scheduleEvent(eventReceiveEnd);
    }

    /**
     * Function that computes a list of channel IDs used by a network node, either for receiving or sending
     *
     * @param node Node for which channel IDs should be computed
     * @param sending True to get the sending channel IDs for the node, false for receiving channel IDs
     * @return List of channel IDs, either for sending or receiving
     */
    @Override
    public List<Integer> computeChannelIDs(NetworkNode node, boolean sending) {
        List<Integer> resultList = new LinkedList<>();
        resultList.add(0);
        return resultList;
    }

    /**
     * Function that computes the index for the modulation and data rate to be used
     *
     * @param node Node for which index should be computed
     * @return Integer index for the modulation and data rate info
     */
    @Override
    public int computeModulationAndDataRateIndex(NetworkNode node) {
        return NetworkSimulator.getInstance().getNetworkSettings().getModulationAndDataRateInfoDefaultIndex();
    }

    /**
     * Function that is called when the network simulation starts, useful for channel objects initialization
     */
    @Override
    public void networkSimulationStart() {}
}