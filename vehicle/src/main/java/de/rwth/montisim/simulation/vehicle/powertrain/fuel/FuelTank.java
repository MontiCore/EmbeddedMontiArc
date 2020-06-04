/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

public class FuelTank {
    public final double capacity;
    public double contained;

    // Init fully loaded
    public FuelTank(FuelPTProperties properties) {
        this.capacity = properties.tankCapacity;
        this.contained = properties.tankCapacity;
    }

    public void setEmpty(){
        this.contained = 0;
    }

    public void setFromPercentage(double percentFull) {
        this.contained = capacity*percentFull*0.01f;
    }

    public void setFullyLoaded() {
        this.contained = capacity;
    }

    public void consume(double amount){
        this.contained -= amount;
        if (this.contained < 0) this.contained = 0;
    }

    public double percentage(){
        return contained/capacity*100;
    }
}