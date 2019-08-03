
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

import java.time.Instant;
import java.util.UUID;


public abstract class EEDiscreteEvent {

    private final Instant eventTime;

    private final EEComponent target;
    
    private final EEDiscreteEventTypeEnum eventType;
    
    private final UUID Id;

    public EEDiscreteEvent(EEDiscreteEventTypeEnum eventType, Instant eventTime, EEComponent target){
        this.eventTime = eventTime;
        this.target = target;
        this.eventType = eventType;
        this.Id = UUID.randomUUID();
    }

    public Instant getEventTime() {
        return eventTime;
    }

    public EEComponent getTarget() {
        return target;
    }
    
    public EEDiscreteEventTypeEnum getEventType() {
    	return this.eventType;
    }
    
    public UUID getId() {
    	return this.Id;
    }
    
    @Override
    public String toString() {
    	return "Type = " + this.eventType +
    			"; Target  = " + this.target +
    			"; Event Time = " +this.eventTime +
    			"; Id = " + this.Id;
    			
    }
}
