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


import simulation.bus.Bus;
import simulation.bus.BusMessage;
import commons.controller.commons.BusEntry;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.time.Duration;
import org.apache.commons.lang3.tuple.Pair;

public class Bridge extends MutableEEComponent{
         protected Pair<Bus, Bus> connected;
         private final Duration delay;

         public Bridge (EESimulator simulator, Pair<Bus, Bus> connected, Duration delay){
             super(simulator, EEComponentType.BRIDGE);
             this.connected = connected;
             this.delay = delay;
             
             //initialize subscribed messages and targetsByMessageId
             Set<BusEntry> subscribedMessages = new HashSet<BusEntry>(connected.getKey().getSubscribedMessages());
             subscribedMessages.addAll(connected.getValue().getSubscribedMessages());
             this.subscribedMessages.addAll(subscribedMessages);
             for(BusEntry messageId : subscribedMessages) {
            	 List<EEComponent> targets = new ArrayList<EEComponent>();
            	 if(connected.getKey().getSubscribedMessages().contains(messageId)) {
            		 targets.add(connected.getKey());
            	 }
            	 if(connected.getValue().getSubscribedMessages().contains(messageId)) {
            		 targets.add(connected.getValue());
            	 }
            	 this.targetsByMessageId.put(messageId, targets);
             }
             
             connected.getKey().registerComponent(this);
             connected.getValue().registerComponent(this);
         }

        public void processEvent(EEDiscreteEvent event){
             if(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE){
                 BusMessage msg = (BusMessage) event;
                 msg.setFinishTime(msg.getEventTime().plus(delay));
                 msg.transmitBytes(msg.getMessageLen(), 0);
                 if(msg.getControllerID() == connected.getLeft().getId()) {
                	 this.getSimulator().addEvent(msg.forwardTo(connected.getRight()));
                 }
                 else if(msg.getControllerID() == connected.getRight().getId()) {
                	 this.getSimulator().addEvent(msg.forwardTo(connected.getLeft()));
                 }
                 else {
                	 throw new IllegalArgumentException("Message from invalid controller" + msg.getControllerID() + "received at: " + this.toString());
                 } 
             }
             else{
                 throw new IllegalArgumentException("Only BusMessages events expected at " + this.toString() + " but was: " + event.getEventType());
             }
        }

         public void update(Bus bus, List<BusEntry> messageIds){
             for(BusEntry messageId : messageIds) {
            	 List<EEComponent> targets = this.targetsByMessageId.getOrDefault(messageId, new ArrayList<EEComponent>());
            	 targets.add(bus);
            	 this.targetsByMessageId.put(messageId, targets);
            	 if(!subscribedMessages.contains(messageId)) {
            		 this.subscribedMessages.add(messageId);
            	 }
             }
        	 if(bus.equals(connected.getKey())){
                 connected.getValue().addSubscribedMessages(this, messageIds);
             }
             else{
                 if(bus.equals(connected.getValue())){
                     connected.getKey().addSubscribedMessages(this, messageIds);
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
