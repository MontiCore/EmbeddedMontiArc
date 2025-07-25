/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;


public class FuelTank {
    final transient FuelPTProperties properties;
    public double contained;

    // Init fully loaded
    public FuelTank(FuelPTProperties properties) {
        this.properties = properties;
        this.contained = properties.tank_capacity;
    }

    public void setEmpty() {
        this.contained = 0;
    }

    public void setFromPercentage(double percentFull) {
        this.contained = properties.tank_capacity * percentFull * 0.01f;
    }

    public void setFullyLoaded() {
        this.contained = properties.tank_capacity;
    }

    public void consume(double amount) {
        this.contained -= amount;
        if (this.contained < 0)
            this.contained = 0;
    }

    public double percentage() {
        return contained / properties.tank_capacity * 100;
    }

}