import commons.controller.commons.BusEntry;
import commons.simulation.DiscreteEvent;

public interface EEComponent {
    /**
     * function that connects unit to the EESimulator and the bus
     */
    void connectToSimulator(EESimulator simulator);

    /**
     * function that transmits data to the bus (via simulator)
     */
    void registerEvent(EESimulator simulator, DiscreteEvent event);

    /**
     * function that receives data from the bus (via simulator)
     */
    void processEvent(DiscreteEvent event);

    /**
     *
     * @return
     */
    BusEntry getID();
}
