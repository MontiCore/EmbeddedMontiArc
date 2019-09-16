/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.visualisationadapter.interfaces;

public enum TrafficSignalStatus {
    RED("RED"),
    GREEN("GREEN"),
    YELLOW("YELLOW");

	private String name;

	private TrafficSignalStatus(String name) {
		this.name = name;
	}

	@Override
	public String toString() {
		return this.name;
	}
}
