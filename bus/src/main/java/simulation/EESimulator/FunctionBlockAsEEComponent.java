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
package simulation.EESimulator;

import com.google.common.collect.Sets;
import commons.controller.commons.BusEntry;
import commons.controller.interfaces.FunctionBlockInterface;
import simulation.bus.BusMessage;

import java.time.Duration;
import java.time.Instant;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FunctionBlockAsEEComponent extends ImmutableEEComponent{

    FunctionBlockInterface functionBlock;

    Map<String, Object> inputs;

    Map<BusEntry, Integer> sizeByMessageId;

    Instant lastExecutionTime;

    public FunctionBlockAsEEComponent(EESimulator simulator, EEComponentType type, List<BusEntry> subscribedMessages,
                                      HashMap<BusEntry, List<EEComponent>> targetsByMessageId, HashMap<BusEntry, Integer> sizeByMessageId,
                                      FunctionBlockInterface functionBlock) {
        super(simulator, type, subscribedMessages, targetsByMessageId);
        //targetsByMessageId and sizeByMessageId contain different keys
        if(Sets.symmetricDifference(targetsByMessageId.keySet(), sizeByMessageId.keySet()).size() != 0){
            throw new IllegalArgumentException("targetsByMessageId and sizeByMessageId must contain the same keys");
        }
        this.functionBlock = functionBlock;
        this.lastExecutionTime = this.getSimulator().getSimulationTime();
        inputs = new HashMap<>();
    }

    @Override
    public void processEvent(EEDiscreteEvent event) {
        if(event.getEventType() != EEDiscreteEventTypeEnum.BUSMESSAGE){
            throw new IllegalArgumentException("FunctionBlockAsEEComponent expects BusMessage as event. Event type was" + event.getEventType());
        }
        BusMessage busMessage = (BusMessage)event;
        inputs.put(busMessage.getMessageID().toString(), busMessage.getMessage());
        //all inputs present
        if (inputs.size() == this.getSubscribedMessages().size()){
            Duration timeDiff = Duration.between(lastExecutionTime, event.getEventTime());

            functionBlock.setInputs(inputs);
            inputs.clear();
            Map<String, Object> outputs = functionBlock.getOutputs();
            Map<String, BusEntry> messagesByName = new HashMap<>();
            for(BusEntry messageId : getTargetsByMessageId().keySet()){
                messagesByName.put(messageId.toString(), messageId);
            }
            functionBlock.execute(timeDiff.toMillis());
            for(Map.Entry<String, Object> entry : outputs.entrySet()){
                if(!messagesByName.containsKey(entry.getKey())){
                    throw new IllegalStateException("Functionblock generated output that cannot be send. Output was: " + entry.getKey());
                }
                this.sendMessage(entry.getValue(), this.sizeByMessageId.get(entry.getKey()), messagesByName.get(entry.getKey()), event.getEventTime());
            }
        }
    }
}
