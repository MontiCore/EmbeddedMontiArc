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
package simulation.network.settings;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObjectType;
import simulation.network.NetworkSettings;
import simulation.network.NetworkSettingsId;
import simulation.network.NetworkTaskId;
import simulation.network.channels.ChannelModelSimple;
import java.util.*;
import static simulation.network.NetworkTaskId.*;

/**
 * Class representing simple network settings
 */
public class SettingsSimple extends NetworkSettings {

    /**
     * Constructor that sets the values for network settings
     */
    public SettingsSimple() {
        setSettingsId(NetworkSettingsId.NETWORK_SETTINGS_ID_SIMPLE);

        setIpv6Prefix("fde938777acb4bd4");
        setIpv6LinkLocalMulticastAddress("ff020000000000000000000000000001");
        setMacBroadcastAddress("ffffffffffff");
        setMacPrefix("fe");

        setSlowDataRateKBits(1000);

        setModulationAndDataRateInfo(new int[][] {
                {3000, 1, 1, 2},
                {4500, 1, 3, 4},
                {6000, 2, 1, 2},
                {9000, 2, 3, 4},
                {12000, 4, 1, 2},
                {18000, 4, 3, 4},
                {24000, 6, 2, 3},
                {27000, 6, 3, 4},
        });
        setModulationAndDataRateInfoDefaultIndex(0);

        setApplicationBeaconUpdateInterval(375000000L);

        setMinimumLocalDelayPerLayer(2000000L / 4L);
        setMaximumLocalDelayPerLayer(3000000L / 4L);

        setMinTaskStartTimeNs(0L);
        setMaxTaskStartTimeNs(3000000000L);

        setMessageBufferSize(30);
        setMessageBufferMaxTime(15000000000L);

        setNetworkChannelModel(new ChannelModelSimple());

        Map<PhysicalObjectType, List<NetworkTaskId>> networkTaskIdMap = new HashMap<>();

        List<NetworkTaskId> taskIdListCars =
            Arrays.asList(NETWORK_TASK_ID_PHY_SIMPLE, NETWORK_TASK_ID_LINK_SIMPLE, NETWORK_TASK_ID_NET_SIMPLE, NETWORK_TASK_ID_TRANSPORT_SIMPLE,
                NETWORK_TASK_ID_APP_BEACON, NETWORK_TASK_ID_APP_MESSAGES_SOFT_STATE, NETWORK_TASK_ID_APP_TRAFFIC_OPTIMIZATION, NETWORK_TASK_ID_APP_VELOCITY_CONTROL);
        networkTaskIdMap.put(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, new LinkedList<>(taskIdListCars));

        setNetworkTaskIdMap(networkTaskIdMap);
    }
}