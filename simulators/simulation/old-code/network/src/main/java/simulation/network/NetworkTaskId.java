/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
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
