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
///**
// *
// * ******************************************************************************
// *  MontiCAR Modeling Family, www.se-rwth.de
// *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
// *  All rights reserved.
// *
// *  This project is free software; you can redistribute it and/or
// *  modify it under the terms of the GNU Lesser General Public
// *  License as published by the Free Software Foundation; either
// *  version 3.0 of the License, or (at your option) any later version.
// *  This library is distributed in the hope that it will be useful,
// *  but WITHOUT ANY WARRANTY; without even the implied warranty of
// *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// *  Lesser General Public License for more details.
// *
// *  You should have received a copy of the GNU Lesser General Public
// *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
// * *******************************************************************************
// */
//package Bus;
//
//
//import commons.controller.commons.BusEntry;
//import simulation.simulator.*;
//
//import java.awt.*;
//import java.util.*;
//import java.lang.Math;
//
//
//public class CANBus implements Bus {
//    /**
//     * dataRate in bit per miliseconds
//     */
//    private int dataRate;
//
//    /**
//     * recognize plugged components
//     */
//    private List<String> components;
//
//    /**
//     * messages that should be transmitted by this bus
//     */
//    private Map<String, BusMessage> activeMessages;
//
//    /**
//     * map with all messages transmitted by this bus
//     */
//    private Map<String, BusMessage> transmittedMessages;
//
//    /**
//     * size overhead in bit
//     */
//    private final int OVERHEAD = 41;
//
//    /**
//     * maximum paylaod per frame in byte
//     */
//    private final int MAX_PAYLOAD_LEN = 8;
//
//    /**
//     *
//     * @param dataRate in bit per milliseconds
//     * @param connections all connected components
//     */
//    public CANBus(int dataRate, Optional[] connections) {
//        if(dataRate <= 0){
//            throw new IllegalArgumentExecption("dataRate has to be positive");
//        }
//        else{
//            this.dataRate = dataRate;
//        }
//
//        this.components = new List<String>;
//        for (int ix = 0; ix < connections.length; ix++) {
//            this.components.add(BusEntry.toString(connections[ix]));
//        }
//    }
//
//    @overried
//    public void registerData(String key, BusMessage msg) {
//        activeMessages.put(key, msg);
//    }
//
//    @override
//    public BusMessage getData(String key) {
//        return Optional.of(transmittedMessages.get(key));
//    }
//
//    @override
//    public Map<String, BusMessage> getAllData() {
//        return transmittedMessages;
//    }
//
//    @override
//    public String[] getImportNames() {
//        Set keys = transmittedMessages.keySet();
//        return (String[]) keys.toArray();
//    }
//
//    @override
//    public void registerComponent(Object component) {
//        this.components.add(BusEntry.toString(component));
//    }
//
//    /**
//     * simulate the transmission of data for a given time
//     *
//     * @param startTime time when simulation starts
//     * @param duration time of the simulation
//     * @return list of transmitted messages
//     */
//    @override
//    public List<BusMessage> simulateFor(int startTime, int duration){
//        List<BusMessages> finishedMessages = new ArrayList<BusMessage>();
//        int finishTime = startTime + duration;
//        int activeTime = startTime;
//
//
//        while (activeMessages != null){
//
//            String nextKey = getNextMessageKey();
//            BusMessage nextMsg = activeMessages.get(nextKey);
//
//
//            //message already should be delivered, so we delete it
//            if(nextMsg.getFinishTime >0 && nextMsg.getFinishTime <startTime){
//                activeMessages.remove(nextKey);
//            }
//
//            else{
//                if(nextMsg.getFinishTime < 0){
//                    nextMsg.setFinishTime(activeTime + getDelay(nextMsg));
//                }
//                activeTime = nextMsg.getFinishTime();
//                if(activeTime > finishTime){
//                    return finishedMessages;
//                }
//                finishedMessages.add(nextMsg);
//                transmittedMessages.add(nextKey, nextMsg);
//                activeMessages.remove(nextKey);
//            }
//        }
//
//        return finishedMessages;
//    }
//
//    /**
//     *
//     * @param msg
//     * @return time message needs to get send
//     */
//    private int getDelay(BusMessage msg) {
//        int allData = msg.getMessageLen()*8 +(int) Math.ceil((double)msg.getMessageLen()/MAX_PAYLOAD_LEN)*OVERHEAD;
//        double delay = allData/ dataRate;
//        delay = Math.ceil(delay);
//        return (int(delay));
//    }
//
//    /**
//     *
//     * @return get key of message with lowest requestTime
//     */
//    private String getNextMessageKey(){
//        String nextKey = "";
//        int nextStartTime = Integer.MAX_VALUE;
//        int nextTime;
//        for(String key: activeMessages){
//            nextTime = (activeMessages.get(key)).getRequestTime();
//            if(nextTime < nextStratTime){
//                nextStartTime = nextTime;
//                nextKey = key;
//            }
//            // if same repuestTime note priority (now: higher in list means higher priority)
//        }
//        return nextKey;
//    }
//}