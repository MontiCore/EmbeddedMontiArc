/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network;

import de.rwth.montisim.commons.simulation.PhysicalObjectType;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

/**
 * Abstract class for settings of the network
 */
public abstract class NetworkSettings {

    /** Constant speed of light in meters per second */
    public static final long SPEED_OF_LIGHT = 299792458L;

    /** Network channel objects that is responsible for the computation of new transmissions */
    private NetworkSettingsId settingsId = NetworkSettingsId.NETWORK_SETTINGS_ID_NONE;

    /** Application beacon update interval */
    private Duration applicationBeaconUpdateInterval = Duration.ofMillis(375L);

    /** IPv6 prefix of network, needs to consist of 16 characters, hex representation */
    private String ipv6Prefix = "fde938777acb4bd4";

    /** IPv6 link local multicast address of network, needs to consist of 32 characters, hex representation */
    private String ipv6LinkLocalMulticastAddress = "ff020000000000000000000000000001";

    /** MAC prefix of network, needs to consist of 2 characters, hex representation */
    private String macPrefix = "fe";

    /** MAC broadcast address of network, needs to consist of 12 characters, hex representation */
    private String macBroadcastAddress = "ffffffffffff";

    /** Initial parts of message are always transmitted at slow data rate (e.g. preamble), measured in kilobits per second */
    private int slowDataRateKBits = 1000;

    /** Minimum value of randomized interval for start time of start tasks */
    private Instant minTaskStartTime = Instant.EPOCH;

    /** Maximum value of randomized interval for start time of start tasks */
    private Instant maxTaskStartTime = Instant.ofEpochSecond(3L);

    /** Maximum amount of messages kept in node buffers */
    private int messageBufferSize = 30;

    /** Maximum amount of time that messages are kept for */
    private Duration messageBufferMaxTime = Duration.ofSeconds(15L);

    /** Integer array that holds the information about available modulation and data rates, data rate in kBits, bits per signal, code rate numerator, code rate denominator */
    private int[][] modulationAndDataRateInfo = new int[][] {
        {12000, 4, 1, 2}
    };

    /** Default index for modulation and data rate info to be used */
    private int modulationAndDataRateInfoDefaultIndex = 0;

    /** Minimum value for random interval for additional local delay between each of four layers */
    private Duration minimumLocalDelayPerLayer = Duration.ofNanos(2000000L / 4L);

    /** Maximum value for random interval for additional local delay between each of four layers */
    private Duration maximumLocalDelayPerLayer = Duration.ofNanos(3000000L / 4L);

    /** Map of physical object types and task lists that are added to the nodes in the network */
    private Map<PhysicalObjectType, List<NetworkTaskId>> networkTaskIdMap = Collections.synchronizedMap(new HashMap<>());

    /** Network channel objects that is responsible for the computation of new transmissions */
    private NetworkChannelModel networkChannelModel = null;

    /**
     * Function that returns settingsId
     *
     * @return Value for settingsId
     */
    public NetworkSettingsId getSettingsId() {
        return settingsId;
    }

    /**
     * Function that sets settingsId
     *
     * @param settingsId New value for settingsId
     */
    protected void setSettingsId(NetworkSettingsId settingsId) {
        this.settingsId = settingsId;
    }

    /**
     * Function that returns applicationBeaconUpdateInterval
     *
     * @return Value for applicationBeaconUpdateInterval
     */
    public Duration getApplicationBeaconUpdateInterval() {
        return applicationBeaconUpdateInterval;
    }

    /**
     * Function that sets applicationBeaconUpdateInterval
     *
     * @param applicationBeaconUpdateInterval New value for applicationBeaconUpdateInterval
     */
    public void setApplicationBeaconUpdateInterval(Duration applicationBeaconUpdateInterval) {
        this.applicationBeaconUpdateInterval = applicationBeaconUpdateInterval;
    }

    /**
     * Get IPv6 prefix of network
     *
     * @return IPv6 prefix of network
     */
    public String getIpv6Prefix() {
        return ipv6Prefix;
    }

    /**
     * Set IPv6 prefix of network
     *
     * @param ipv6Prefix New IPv6 prefix of network
     */
    protected void setIpv6Prefix(String ipv6Prefix) {
        this.ipv6Prefix = ipv6Prefix;
    }

    /**
     * Function that returns ipv6LinkLocalMulticastAddress
     *
     * @return Value for ipv6LinkLocalMulticastAddress
     */
    public String getIpv6LinkLocalMulticastAddress() {
        return ipv6LinkLocalMulticastAddress;
    }

    /**
     * Function that sets ipv6LinkLocalMulticastAddress
     *
     * @param ipv6LinkLocalMulticastAddress New value for ipv6LinkLocalMulticastAddress
     */
    protected void setIpv6LinkLocalMulticastAddress(String ipv6LinkLocalMulticastAddress) {
        this.ipv6LinkLocalMulticastAddress = ipv6LinkLocalMulticastAddress;
    }

    /**
     * Get MAC prefix of network
     *
     * @return MAC prefix of network
     */
    public String getMacPrefix() {
        return macPrefix;
    }

    /**
     * Set MAC prefix of network
     *
     * @param macPrefix New MAC prefix of network
     */
    protected void setMacPrefix(String macPrefix) {
        this.macPrefix = macPrefix;
    }

    /**
     * Get MAC broadcast address of network
     *
     * @return MAC broadcast address of network
     */
    public String getMacBroadcastAddress() {
        return macBroadcastAddress;
    }

    /**
     * Set MAC broadcast address of network
     *
     * @param macBroadcastAddress New MAC broadcast address of network
     */
    protected void setMacBroadcastAddress(String macBroadcastAddress) {
        this.macBroadcastAddress = macBroadcastAddress;
    }

    /**
     * Get preamble data rate of network
     *
     * @return Preamble data rate of network
     */
    public int getSlowDataRateKBits() {
        return slowDataRateKBits;
    }

    /**
     * Set preamble data rate of network
     *
     * @param slowDataRateKBits New preamble data rate of network
     */
    protected void setSlowDataRateKBits(int slowDataRateKBits) {
        this.slowDataRateKBits = slowDataRateKBits;
    }

    /**
     * Get minimum task start time
     *
     * @return Minimum task start time
     */
    public Instant getMinTaskStartTime() {
        return minTaskStartTime;
    }

    /**
     * Set minimum task start time
     *
     * @param minTaskStartTime New minimum task start time
     */
    protected void setMinTaskStartTime(Instant minTaskStartTime) {
        this.minTaskStartTime = minTaskStartTime;
    }

    /**
     * Get maximum task start time
     *
     * @return Maximum task start time
     */
    public Instant getMaxTaskStartTime() {
        return maxTaskStartTime;
    }

    /**
     * Set maximum task start time
     *
     * @param maxTaskStartTime New maximum task start time
     */
    protected void setMaxTaskStartTime(Instant maxTaskStartTime) {
        this.maxTaskStartTime = maxTaskStartTime;
    }

    /**
     * Function that returns messageBufferSize
     *
     * @return Value for messageBufferSize
     */
    public int getMessageBufferSize() {
        return messageBufferSize;
    }

    /**
     * Function that sets messageBufferSize
     *
     * @param messageBufferSize New value for messageBufferSize
     */
    protected void setMessageBufferSize(int messageBufferSize) {
        this.messageBufferSize = messageBufferSize;
    }

    /**
     * Function that returns messageBufferMaxTime
     *
     * @return Value for messageBufferMaxTime
     */
    public Duration getMessageBufferMaxTime() {
        return messageBufferMaxTime;
    }

    /**
     * Function that sets messageBufferMaxTime
     *
     * @param messageBufferMaxTime New value for messageBufferMaxTime
     */
    protected void setMessageBufferMaxTime(Duration messageBufferMaxTime) {
        this.messageBufferMaxTime = messageBufferMaxTime;
    }

    /**
     * Function that returns modulationAndDataRateInfo
     *
     * @return Value for modulationAndDataRateInfo
     */
    public int[][] getModulationAndDataRateInfo() {
        return modulationAndDataRateInfo;
    }

    /**
     * Function that sets modulationAndDataRateInfo
     *
     * @param modulationAndDataRateInfo New value for modulationAndDataRateInfo
     */
    public void setModulationAndDataRateInfo(int[][] modulationAndDataRateInfo) {
        this.modulationAndDataRateInfo = modulationAndDataRateInfo;
    }

    /**
     * Function that returns modulationAndDataRateInfoDefaultIndex
     *
     * @return Value for modulationAndDataRateInfoDefaultIndex
     */
    public int getModulationAndDataRateInfoDefaultIndex() {
        return modulationAndDataRateInfoDefaultIndex;
    }

    /**
     * Function that sets modulationAndDataRateInfoDefaultIndex
     *
     * @param modulationAndDataRateInfoDefaultIndex New value for modulationAndDataRateInfoDefaultIndex
     */
    public void setModulationAndDataRateInfoDefaultIndex(int modulationAndDataRateInfoDefaultIndex) {
        this.modulationAndDataRateInfoDefaultIndex = modulationAndDataRateInfoDefaultIndex;
    }

    /**
     * Function that returns minimumLocalDelayPerLayer
     *
     * @return Value for minimumLocalDelayPerLayer
     */
    public Duration getMinimumLocalDelayPerLayer() {
        return minimumLocalDelayPerLayer;
    }

    /**
     * Function that sets minimumLocalDelayPerLayer
     *
     * @param minimumLocalDelayPerLayer New value for minimumLocalDelayPerLayer
     */
    public void setMinimumLocalDelayPerLayer(Duration minimumLocalDelayPerLayer) {
        this.minimumLocalDelayPerLayer = minimumLocalDelayPerLayer;
    }

    /**
     * Function that returns maximumLocalDelayPerLayer
     *
     * @return Value for maximumLocalDelayPerLayer
     */
    public Duration getMaximumLocalDelayPerLayer() {
        return maximumLocalDelayPerLayer;
    }

    /**
     * Function that sets maximumLocalDelayPerLayer
     *
     * @param maximumLocalDelayPerLayer New value for maximumLocalDelayPerLayer
     */
    public void setMaximumLocalDelayPerLayer(Duration maximumLocalDelayPerLayer) {
        this.maximumLocalDelayPerLayer = maximumLocalDelayPerLayer;
    }

    /**
     * Get map of node network tasks
     *
     * @return Map of node network tasks
     */
    public Map<PhysicalObjectType, List<NetworkTaskId>> getNetworkTaskIdMap() {
        return Collections.synchronizedMap(new HashMap<>(networkTaskIdMap));
    }

    /**
     * Set map of node network tasks
     *
     * @param networkTaskIdMap New map of node network tasks
     */
    protected void setNetworkTaskIdMap(Map<PhysicalObjectType, List<NetworkTaskId>> networkTaskIdMap) {
        this.networkTaskIdMap = Collections.synchronizedMap(new HashMap<>(networkTaskIdMap));
    }

    /**
     * Get network channel objects
     *
     * @return Network channel objects
     */
    public NetworkChannelModel getNetworkChannelModel() {
        return networkChannelModel;
    }

    /**
     * Set network channel objects
     *
     * @param networkChannelModel New network channel objects
     */
    protected void setNetworkChannelModel(NetworkChannelModel networkChannelModel) {
        this.networkChannelModel = networkChannelModel;
    }

    /**
     * Improved toString() method to get more information
     *
     * @return String of information about object
     */
    @Override
    public String toString() {
        return "NetworkSettings{" +
                "settingsId=" + settingsId +
                ", applicationBeaconUpdateInterval=" + applicationBeaconUpdateInterval +
                ", ipv6Prefix='" + ipv6Prefix + '\'' +
                ", ipv6LinkLocalMulticastAddress='" + ipv6LinkLocalMulticastAddress + '\'' +
                ", macPrefix='" + macPrefix + '\'' +
                ", macBroadcastAddress='" + macBroadcastAddress + '\'' +
                ", slowDataRateKBits=" + slowDataRateKBits +
                ", minTaskStartTimeNs=" + minTaskStartTime +
                ", maxTaskStartTimeNs=" + maxTaskStartTime +
                ", messageBufferSize=" + messageBufferSize +
                ", messageBufferMaxTime=" + messageBufferMaxTime +
                ", modulationAndDataRateInfo=" + Arrays.toString(modulationAndDataRateInfo) +
                ", modulationAndDataRateInfoDefaultIndex=" + modulationAndDataRateInfoDefaultIndex +
                ", minimumLocalDelayPerLayer=" + minimumLocalDelayPerLayer +
                ", maximumLocalDelayPerLayer=" + maximumLocalDelayPerLayer +
                ", networkTaskIdMap=" + networkTaskIdMap +
                ", networkChannelModel=" + networkChannelModel +
                '}';
    }
}
