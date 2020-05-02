/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain.fuel;

public class FuelTank {
    public final double capacity;
    public double contained;

    public static FuelTank newFullyLoaded(double capacity){
        return new FuelTank(capacity, capacity);
    }

    public static FuelTank newEmpty(double capacity){
        return new FuelTank(capacity, 0);
    }

    public static FuelTank newFromPercent(double capacity, double percentFull){
        return new FuelTank(capacity, capacity*percentFull*0.01f);
    }

    public FuelTank(double capacity, double contained) {
        this.capacity = capacity;
        this.contained = contained;
    }

    public void consume(double amount){
        this.contained -= amount;
        if (this.contained < 0) this.contained = 0;
    }

    public double percentage(){
        return contained/capacity*100;
    }
}