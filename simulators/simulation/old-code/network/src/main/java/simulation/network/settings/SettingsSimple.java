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
import simulation.network.channels.ChannelModelSimple;

import java.time.Duration;
import java.time.Instant;
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

        setModulationAndDataRateInfo(new int[][]{
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

        setApplicationBeaconUpdateInterval(Duration.ofMillis(375L));

        setMinimumLocalDelayPerLayer(Duration.ofNanos(2000000L / 4L));
        setMaximumLocalDelayPerLayer(Duration.ofNanos(3000000L / 4L));

        setMinTaskStartTime(Instant.EPOCH);
        setMaxTaskStartTime(Instant.ofEpochSecond(3L));

        setMessageBufferSize(30);
        setMessageBufferMaxTime(Duration.ofSeconds(15L));

        setNetworkChannelModel(new ChannelModelSimple());

        Map<PhysicalObjectType, List<NetworkTaskId>> networkTaskIdMap = new HashMap<>();

        List<NetworkTaskId> taskIdListCars =
                Arrays.asList(NETWORK_TASK_ID_PHY_SIMPLE, NETWORK_TASK_ID_LINK_SIMPLE, NETWORK_TASK_ID_NET_SIMPLE, NETWORK_TASK_ID_TRANSPORT_SIMPLE,
                        NETWORK_TASK_ID_APP_BEACON, NETWORK_TASK_ID_APP_MESSAGES_SOFT_STATE, NETWORK_TASK_ID_APP_TRAFFIC_OPTIMIZATION, NETWORK_TASK_ID_APP_VELOCITY_CONTROL);
        networkTaskIdMap.put(PhysicalObjectType.PHYSICAL_OBJECT_TYPE_CAR, new LinkedList<>(taskIdListCars));

        setNetworkTaskIdMap(networkTaskIdMap);
    }
}
