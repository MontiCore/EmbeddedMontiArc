
package simulation.bus; 

import java.util.Optional;

import com.google.common.base.Function;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import simulation.EESimulator.EEComponent;

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
