public class RightFrontWheelDistanceToStreetSensor extends AbstractDistanceSensor {
    @Override
    protected Double calculateDistance(IPhysicalVehicle physicalVehicle) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceFrontRightWheelToRightStreetBorder(physicalVehicle).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

}
