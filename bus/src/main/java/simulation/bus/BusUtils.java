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

package simulation.bus; 

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.Stack;
import java.util.UUID;

import org.apache.commons.lang3.tuple.Pair;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;

import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;

public class BusUtils {

	static Collection<Bus> findConnectedBuses(Bus bus) {
		
		Predicate<EEComponent> bridgeFilter = new Predicate<EEComponent>() {
			public boolean apply(EEComponent comp) {
				return comp.getComponentType() == EEComponentType.BRIDGE;
			}
		};
		
		Map<UUID, Bus> busById = new HashMap<UUID, Bus>();
		busById.put(bus.getId(), bus);
		Stack<Bus> workStack = new Stack<Bus>();
		workStack.add(bus);
		
		while(!workStack.isEmpty()) {
			Bus cur = workStack.pop();
			Iterable<EEComponent> bridges = Iterables.filter(cur.getConnectedComponents(), bridgeFilter);
			for(EEComponent bridge : bridges) {
				Pair<Bus, Bus> busPair = ((Bridge)bridge).getConnectedBuses();
				if(!busById.containsKey(busPair.getLeft().getId())) {
					busById.put(busPair.getLeft().getId(), busPair.getLeft());
					workStack.add(busPair.getLeft());
				}
				if(!busById.containsKey(busPair.getRight().getId())) {
					busById.put(busPair.getRight().getId(), busPair.getRight());
					workStack.add(busPair.getRight());
				}
			}
		}
		busById.remove(bus.getId());
		return busById.values();
	}
	
	public static Set<Bus> findConnectedBuses(EEComponent component) {		
		Set<Bus> connectedBuses = new HashSet<Bus>();
		for(List<EEComponent> targets : component.getTargetsByMessageId().values()) {
			for(Bus bus : Iterables.filter(targets, Bus.class)) {
				connectedBuses.add(bus);
			}
		}
		return connectedBuses;
	}
	
	static Optional<EEComponent> findComponentWithID(Iterable<EEComponent> components, UUID ID) {
		EEComponent comp = Iterables.tryFind(components, 
				new Predicate<EEComponent>() {
					public boolean apply(EEComponent comp) {
						return comp.getId().equals(ID);
					}
		}).orNull();
		return Optional.ofNullable(comp);
	}

}
