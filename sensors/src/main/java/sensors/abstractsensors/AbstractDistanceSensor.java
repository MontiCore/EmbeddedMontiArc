/* (c) https://github.com/MontiCore/monticore */
package sensors.abstractsensors;

import commons.simulation.IPhysicalVehicle;
import simulation.vehicle.PhysicalVehicle;

/**
 * Created by Aklima Zaman on 2/8/2017.
 */
public abstract class AbstractDistanceSensor extends AbstractSensor {
    private Double value;

    public AbstractDistanceSensor(PhysicalVehicle physicalVehicle) {
        super(physicalVehicle);
    }

    @Override
    protected void calculateValue() {
        this.value = calculateDistance(getPhysicalVehicle());
    }

    protected abstract Double calculateDistance(IPhysicalVehicle v);

    @Override
    public Double getValue() {
        return this.value;
    }

    @Override
    public String getTypeName() {
        return Double.class.getTypeName();
    }
}
