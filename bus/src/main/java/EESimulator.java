import simulation.vehicle.VehicleActuator;
import sun.management.Sensor;

import java.time.Duration;
import java.time.Instant;
import java.util.*;
import java.util.Comparator;

import commons.simulation.DiscreteEvent;


public class EESimulator {

    /** time of simulation          */
    private long simulationTimeNs = 0;

    /** time between last simulation step     */
    private long deltaSimulationTimeNs = 0;

    /**
     * list of discrete events (in the future)
     */
    private static final EESimulatorComparator listComparator = new EESimulatorComparator();
    private PriorityQueue<DiscreteEvent> eventList = new PriorityQueue<>(Integer.MAX_VALUE, listComparator);

    /**
     * lists of objects that can get addressed by messages
     */
    private List<EEComponent> busList = Collections.synchronizedList(new LinkedList<>());

    private List<EEComponent> sensorList = Collections.synchronizedList(new LinkedList<>());

    private List<EEComponent> actuatorList = Collections.synchronizedList(new LinkedList<>());


    /**
     * function that adds event to eventList
     * @param event event to add
     */

    public void addEvent(DiscreteEvent event){
        eventList.offer(event);
    }


    /**
     * simulate until eventList is empty or next event is in future
     */
    public void simulateNextTick(Instant actualTime, Duration tick){                 //add time

        for (DiscreteEvent event: eventList) {
            if(event.getType() == MessageType.SEND){                //waiting for the updated bus interface
                //TODO: send this event to bus
                

            } else if(event.getType() == MessageType.RECEIVE){
                //TODO: send this event to actuator or next bus
            }

            
        }
    }

    /**
     * function that adds component to listenerList
     * @param listener component to add
     */
    public void registerComponent(EEComponent listener){
        if (listener instanceof Bus){
            busList.add(listener);
        }
        else if (listener instanceof VehicleActuator) {
            actuatorList.add(listener);
        } else if (listener instanceof Sensor){
            sensorList.add(listener);
        } else { return;}
    }

    public long getDeltaSimulationTimeNs() {
        return deltaSimulationTimeNs;
    }

    public long getSimulationTimeNs() {
        return simulationTimeNs;
    }


}

class EESimulatorComparator implements Comparator<DiscreteEvent>
{
    // Used for sorting in ascending order of
    public int compare(DiscreteEvent a, DiscreteEvent b)
    {
        if (a.getRequestTime() < b.getRequestTime()) {return -1;}
        else if (a.getRequestTime() == b.getRequestTime()) {return 0;}
        else {return 1;}
    }
}

