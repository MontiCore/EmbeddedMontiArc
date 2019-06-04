/**
 *
 *  ******************************************************************************
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

//package Bus;


import commons.controller.commons.BusEntry;
import simulation.simulator.*;

import java.awt.*;
import java.util.*;
import java.lang.Math;
import java.util.List;
import java.util.ArrayList;

public class CanBus extends Bus {

    /**
     * dataRate in bit per miliseconds
     */
    private int dataRate;


    /**
     * messages that should be transmitted by this bus
     */
    private List<BusMessage> activeMessages;

    /**
     * map with all messages transmitted by this bus
     */
    private List<BusMessage> transmittedMessages;

    /**
     * size overhead in bit
     */
    private final int OVERHEAD = 41;

    /**
     * maximum paylaod per frame in byte
     */
    private final int MAX_PAYLOAD_LEN = 8;


    /**
     *
     * @param dataRate
     */
    public CanBus(int dataRate) {
        if(dataRate <= 0){
            throw new IllegalArgumentException("dataRate has to be positive");
        }
        else{
            this.dataRate = dataRate;
        }
    }

    

    /**
     * Simulate the transmission of data for a given time
     * @param simulationObjects
     * @param totalTime
     * @param deltaTime
     */
    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {

        List<BusMessage> finishedMessages = new ArrayList<BusMessage>();
        long finishTime = totalTime + deltaTime;
        long activeTime = totalTime;
        activeMessages = getIncomingMessagesUnitl(finishTime);

        while (activeMessages != null){

            //String nextKey = getNextMessageKey();
            BusMessage nextMsg = getNextMessage();


            //message already should be delivered, so we delete it
            if(nextMsg.getFinishTime() >0 && nextMsg.getFinishTime() < totalTime){
                activeMessages.remove(nextMsg);
            }

            else{
                if(nextMsg.getFinishTime() < 0){
                    nextMsg.setFinishTime(activeTime + getDelay(nextMsg));
                }
                activeTime = nextMsg.getFinishTime();
                if(activeTime > finishTime){
                    break;
                }
                finishedMessages.add(nextMsg);
                transmittedMessages.add(nextMsg);
                activeMessages.remove(nextMsg);

                this.scheduleEvent(new BusMessageDeliveredEvent(nextMsg));
            }
        }

        super.didExecuteLoop(simulationObjects, totalTime, deltaTime);
    }


    /**
     * @param finalTime time in nanoseconds
     * @return all incoming messages until time finalTime
     */
    List<BusMessage> getIncomingMessagesUnitl(long finalTime) {
        List<BusMessage> res = new ArrayList<BusMessage>();
        List<DiscreteEvent> events = this.getEventList();
        Optional<BusMessageTransmissionRequestEvent> transmissionReq = Optional.empty();
        if (!events.isEmpty()) {
            DiscreteEvent event = events.get(0);
            if (event.getEventTime() <= finalTime && event instanceof BusMessageTransmissionRequestEvent) {
                transmissionReq = Optional.of((BusMessageTransmissionRequestEvent) event);
                activeMessages.add(transmissionReq.get().getMessage());
                res.add(transmissionReq.get().getMessage());
            } else {
                transmissionReq = Optional.empty();
                if (!(event instanceof BusMessageTransmissionRequestEvent)) {
                    // TODO: error!
                }
            }
        }
        while (!events.isEmpty() && transmissionReq.isPresent()) {
            DiscreteEvent event = events.get(0);
            if (event.getEventTime() <= finalTime && event instanceof BusMessageTransmissionRequestEvent) {
                transmissionReq = Optional.of((BusMessageTransmissionRequestEvent) event);
                activeMessages.add(transmissionReq.get().getMessage());
                res.add(transmissionReq.get().getMessage());
            } else {
                transmissionReq = Optional.empty();
                if (!(event instanceof BusMessageTransmissionRequestEvent)) {
                    // TODO: error!
                }
            }
        }
        this.setEventList(events);
        return res;
    }




    /**
     *
     * @param msg
     * @return time message needs to get send
     */
    private int getDelay(BusMessage msg) {
        int allData = msg.getMessageLen()*8 +(int) Math.ceil((double)msg.getMessageLen()/MAX_PAYLOAD_LEN)*OVERHEAD;
        double delay = allData/ dataRate;
        delay = Math.ceil(delay);
        return (int)delay;
    }

    /**
     *
     * @return get key of message with lowest requestTime
     */
    private BusMessage getNextMessage(){
        BusMessage nextMsg = null;
        int nextStartTime = Integer.MAX_VALUE;
        int nextTime;
        for(BusMessage msg: activeMessages){
            nextTime = msg.getRequestTime();
            if(nextTime < nextStartTime){
                nextStartTime = nextTime;
                nextMsg = msg;
            }
        }
        return nextMsg;
    }
}