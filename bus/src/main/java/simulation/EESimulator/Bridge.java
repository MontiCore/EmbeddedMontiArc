package simulation.EESimulator;
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
import simulation.bus.Bus;
import simulation.bus.BusMessage;
import commons.controller.commons.BusEntry;

import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Set;
import java.util.UUID;
import java.util.ArrayList;
import java.util.List;

import java.time.Duration;
import java.time.Instant;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;

public class Bridge extends EEComponent{
         protected Pair<Bus, Bus> connected;
         private final Duration delay;
         private UUID id;

         public Bridge (EESimulator simulator, Pair<Bus, Bus> connected, Duration delay){
             super(simulator);
             componentType = EEComponentType.BRIDGE;
             this.connected = connected;
             this.delay = delay;
             this.id = UUID.randomUUID();
             connected.getKey().registerComponent(this);
             connected.getValue().registerComponent(this);
         }

         public UUID getID(){
             return this.id;
         }

        public void processEvent(EEDiscreteEvent event){
             if(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE){
                 BusMessage msg = (BusMessage) event;
                 if(msg.getControllerID() == connected.getLeft().getID()) {
                	 msg.forwardTo(connected.getRight());
                 }
                 else if(msg.getControllerID() == connected.getRight().getID()) {
                	 msg.forwardTo(connected.getLeft());
                 }
                 else {
                	 throw new IllegalArgumentException("Message from invalid controller" + msg.getControllerID() + "received at: " + this.toString());
                 } 
             }
             else{
                 throw new IllegalArgumentException("Only BusMessages events expected at " + this.toString() + " but was: " + event.getEventType());
             }
        }

         public void update(Bus bus, List<BusEntry> messages){
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
         
         public Pair<Bus, Bus> getConnectedBuses() {
        	 return this.connected;
         }

}
