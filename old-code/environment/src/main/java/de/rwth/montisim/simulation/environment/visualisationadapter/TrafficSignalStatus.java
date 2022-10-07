/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

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
