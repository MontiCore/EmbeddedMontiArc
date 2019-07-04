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
import java.util.Optional;

import com.google.common.base.Function;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;

public class BusUtils {

	static Iterable<Bus> findConnectedBuses(Iterable<EEComponent> components) {
		Predicate<EEComponent> busFilter = new Predicate<EEComponent>() {
			public boolean apply(EEComponent comp) {
				return comp instanceof Bus;
			}
		};
		Iterable<EEComponent> comps = Iterables.filter(components, busFilter);

	    Function<EEComponent, Bus> castToBus = new Function<EEComponent, Bus>() {
	        @Override
	        public Bus apply(EEComponent input) {
	            return (Bus) input;
	        }
	    };
		Iterable<Bus> buses = Iterables.transform(comps, castToBus);
		return buses;
	}
	
	static Optional<EEComponent> findComponentWithID(Iterable<EEComponent> components, String ID) {
		EEComponent comp = Iterables.tryFind(components, 
				new Predicate<EEComponent>() {
					public boolean apply(EEComponent comp) {
						return comp.getID().equals(ID);
					}
		}).orNull();
		return Optional.ofNullable(comp);
	}

}
