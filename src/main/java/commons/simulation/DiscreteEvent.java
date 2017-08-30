package commons.simulation;

/**
 * Interface for a discrete event in a discrete event simulation
 */
public interface DiscreteEvent {

    /**
     * Function that returns the time of the event
     *
     * @return Time of the event
     */
    public long getEventTime();

    /**
     * Function that returns a numeric identifier for the event
     *
     * @return Numeric identifier for the event
     */
    public int getEventId();
}
