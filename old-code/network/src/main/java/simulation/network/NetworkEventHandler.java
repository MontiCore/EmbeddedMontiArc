/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network;

/**
 * Interface that enforces implementation of a method to handle network events
 */
public interface NetworkEventHandler {

    /**
     * Function that handles network events
     */
    public void handleNetworkEvent(NetworkDiscreteEvent event);
}
