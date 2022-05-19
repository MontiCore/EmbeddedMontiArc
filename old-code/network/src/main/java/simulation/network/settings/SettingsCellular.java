/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network.settings;

import de.rwth.montisim.commons.simulation.PhysicalObjectType;
import simulation.network.NetworkSettings;
import simulation.network.NetworkSettingsId;
import simulation.network.NetworkTaskId;
import simulation.network.channels.ChannelModelCellular;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static simulation.network.NetworkTaskId.*;

/**
 * Class representing network settings for indirect vehicle to vehicle communication via cellular network
 */
public class SettingsCellular extends NetworkSettings {

    /**
     * Constructor that sets the values for network settings
     */
    public SettingsCellular() {
        setSettingsId(NetworkSettingsId.NETWORK_SETTINGS_ID_CELLULAR);

        setIpv6Prefix("fde938777acb4bd4");
        setIpv6LinkLocalMulticastAddress("ff020000000000000000000000000001");
        setMacBroadcastAddress("");
        setMacPrefix("");

        setSlowDataRateKBits(1);

        setModulationAndDataRateInfo(new int[][]{
                // Uplink 10 MHz
                {147, 2, 308, 1024},
                {565, 2, 449, 1024},
                {1679, 2, 602, 1024},
                {1489, 4, 490, 1024},
                {3335, 4, 616, 1024},
                {4517, 6, 772, 1024},
                {7796, 6, 873, 1024},
                {12531, 6, 948, 1024},

                // Downlink 10 MHz
                {416, 2, 308, 1024},
                {1589, 2, 449, 1024},
                {4698, 2, 602, 1024},
                {4192, 4, 490, 1024},
                {9182, 4, 616, 1024},
                {12376, 6, 772, 1024},
                {21702, 6, 873, 1024},
                {33972, 6, 948, 1024},
        });

        // Must be set to an uplink index, channel objects shifts it to downlink when needed
        setModulationAndDataRateInfoDefaultIndex(0);

        setApplicationBeaconUpdateInterval(Duration.ofMillis(375L));

        setMinimumLocalDelayPerLayer(Duration.ofNanos(2500000L / 4L));
        setMaximumLocalDelayPerLayer(Duration.ofNanos(3500000L / 4L));

        setMinTaskStartTime(Instant.EPOCH);
        setMaxTaskStartTime(Instant.ofEpochSecond(3L));

        setMessageBufferSize(30);
        setMessageBufferMaxTime(Duration.ofSeconds(15L));

        setNetworkChannelModel(new ChannelModelCellular());

        Map<PhysicalObjectType, List<NetworkTaskId>> networkTaskIdMap = new HashMap<>();

        List<NetworkTaskId> taskIdListCars =
                Arrays.asList(NETWORK_TASK_ID_PHY_INTERFERENCE, NETWORK_TASK_ID_LINK_BUFFERED_ROHC, NETWORK_TASK_ID_NET_SIMPLE, NETWORK_TASK_ID_TRANSPORT_SIMPLE,
                        NETWORK_TASK_ID_APP_BEACON, NETWORK_TASK_ID_APP_MESSAGES_SOFT_STATE, NETWORK_TASK_ID_APP_TRAFFIC_OPTIMIZATION, NETWORK_TASK_ID_APP_VELOCITY_CONTROL);
        networkTaskIdMap.put(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, new LinkedList<>(taskIdListCars));

        List<NetworkTaskId> taskIdListBaseStations =
                Arrays.asList(NETWORK_TASK_ID_PHY_INTERFERENCE, NETWORK_TASK_ID_LINK_BUFFERED_ROHC, NETWORK_TASK_ID_NET_CELLULAR_MULTICAST);
        networkTaskIdMap.put(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_NETWORK_CELL_BASE_STATION, new LinkedList<>(taskIdListBaseStations));

        setNetworkTaskIdMap(networkTaskIdMap);
    }
}
