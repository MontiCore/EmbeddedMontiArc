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
import java.time.Duration;
import java.time.Instant;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import commons.simulation.DiscreteEvent;


public class EESimulator {

    /** time of simulation          */
    private long simulationTimeNs = 0;

    /** time between last simulation step     */
    private long deltaSimulationTimeNs = 0;

    /**
     * list of discrete events (in the future)
     */
    private List<DiscreteEvent> eventList = Collections.synchronizedList(new LinkedList<>()); //change to prio queue

    /**
     * list of objects that can get addressed by messages
     */
    private List<EEComponent> listenerList = Collections.synchronizedList(new LinkedList<>());      //split


    /**
     * function that adds event to eventList
     * @param event event to add
     */
    public void addEvent(DiscreteEvent event){          //change name to schedule event?
//        //event in the past
//        if(event.getRequestTime() < simulationTimeNs){
//            return;
//        }
//
//        int index = 0;
//        for(DiscreteEvent e : eventList){
//            if(e.getRequestTime() > event.getRequestTime()){
//                break;
//            }
//            index++;
//        }
//        eventList.add(index, event);

    }


    /**
     * simulate until eventList is empty or next event is in future
     */
    public void simulateNextTick(Instant actualTime, Duration tick){                 //add time

//        for (DiscreteEvent event: eventList) {
//            if(event.getType() == MessageType.SEND){                //waiting for the updated bus interface
//                //TODO: send this event to bus
//
//
//            } else if(event.getType() == MessageType.RECEIVE){
//                eventList.get(0).getTarget().receiveData(eventList.get(0).getData());
//                eventList.remove(0);
//            }
//            simulationTimeNs = event.getRequestTime();
//            
//        }
    }

    /**
     * function that adds component to listenerList
     * @param listener component to add
     */
    public void registerComponent(EEComponent listener){
        listenerList.add(listener);
    }

    public long getDeltaSimulationTimeNs() {
        return deltaSimulationTimeNs;
    }

    public long getSimulationTimeNs() {
        return simulationTimeNs;
    }


}
