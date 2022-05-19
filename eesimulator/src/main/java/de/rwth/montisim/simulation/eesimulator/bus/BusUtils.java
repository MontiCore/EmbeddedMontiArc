/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus;

public class BusUtils {

    // TODO
    // static Collection<Bus> findConnectedBuses(Bus bus) {

    // 	Predicate<EEComponent> bridgeFilter = new Predicate<EEComponent>() {
    // 		public boolean apply(EEComponent comp) {
    // 			return comp.getComponentType() == EEComponentType.BRIDGE;
    // 		}
    // 	};

    // 	Map<UUID, Bus> busById = new HashMap<UUID, Bus>();
    // 	busById.put(bus.getId(), bus);
    // 	Stack<Bus> workStack = new Stack<Bus>();
    // 	workStack.add(bus);

    // 	while(!workStack.isEmpty()) {
    // 		Bus cur = workStack.pop();
    // 		Iterable<EEComponent> bridges = Iterables.filter(cur.getConnectedComponents(), bridgeFilter);
    // 		for(EEComponent bridge : bridges) {
    // 			Pair<Bus, Bus> busPair = ((Bridge)bridge).getConnectedBuses();
    // 			if(!busById.containsKey(busPair.getLeft().getId())) {
    // 				busById.put(busPair.getLeft().getId(), busPair.getLeft());
    // 				workStack.add(busPair.getLeft());
    // 			}
    // 			if(!busById.containsKey(busPair.getRight().getId())) {
    // 				busById.put(busPair.getRight().getId(), busPair.getRight());
    // 				workStack.add(busPair.getRight());
    // 			}
    // 		}
    // 	}
    // 	busById.remove(bus.getId());
    // 	return busById.values();
    // }

    // public static Set<Bus> findConnectedBuses(EEComponent component) {
    // 	Set<Bus> connectedBuses = new HashSet<Bus>();
    // 	for(List<EEComponent> targets : component.getTargetsByMessageId().values()) {
    // 		for(Bus bus : Iterables.filter(targets, Bus.class)) {
    // 			connectedBuses.add(bus);
    // 		}
    // 	}
    // 	return connectedBuses;
    // }

    // static Optional<EEComponent> findComponentWithID(Iterable<EEComponent> components, UUID ID) {
    // 	EEComponent comp = Iterables.tryFind(components,
    // 			new Predicate<EEComponent>() {
    // 				public boolean apply(EEComponent comp) {
    // 					return comp.getId().equals(ID);
    // 				}
    // 	}).orNull();
    // 	return Optional.ofNullable(comp);
    // }

}
