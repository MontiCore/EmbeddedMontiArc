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


import java.lang.Object;
import commons.simulation.DiscreteEvent;
import commons.controller.commons.BusEntry;

import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Set;

import java.util.ArrayList;
import java.util.List;

import java.time.Duration;
import java.time.Instant;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;

public class Bridge extends EEComponent{
         protected Pair<Bus, Bus> connected;
         private final Instant delay;

         public Bridge (EESimulator simulator, Pair<Bus, Bus> connected, Instant delay){
             super(simulator);
             componentType = EEComponentType.BRIDGE;
             this.connected = connected;
             List<BusEntry> listenConnected1 = connected.getKey().getListenTo();
             List<BusEntry> listenConnected2 = connected.getValue().getListenTo();
             connected.getKey().registerComponent(this, listenConnected2);
             connected.getValue().registerComponent(this, listenConnected1);
             this.delay = delay;

         }

         public String getID(){
             return componentType.toString();
         }

        public void processEvent(DiscreteEvent event){
             if(event instanceof BusMessage){
                 BusMessage msg = (BusMessage) event;
                 if (msg.getType() == MessageType.SEND) {
                    //need to know which Bus send this event so to which one I have to send
                 } else {
                     throw new IllegalArgumentException(
                             "Invalid MessageType. Expected SEND but was " + msg.getType().toString());
                 }
             }
             else{
                 throw new IllegalArgumentException("Only BusMessages expected.");
             }
        }

         protected void update(Bus bus, List<BusEntry> messages){
             if(bus.equals(connected.getKey())){
                 connected.getValue().updateSendTo(this, messages);
             }
             else{
                 if(bus.equals(connected.getValue())){
                     connected.getKey().updateSendTo(this, messages);
                 }
                 else{
                     throw new IllegalArgumentException("Message send by unknown Bus.");
                 }
             }
         }

}
