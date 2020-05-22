package de.rwth.montisim.simulation.vehicle.vehicleproperties;

public abstract class PowerTrainProperties {
    
    public static enum TractionType {
        FRONT,
        REAR,
        ALL // 4x4
    }
    public TractionType tractionType = TractionType.REAR;
    public TractionType brakingType = TractionType.REAR;

    // TODO check
    public double maxBrakingForce = 5000; // In Newtons

}