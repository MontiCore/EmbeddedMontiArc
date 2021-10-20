public class RightFrontDistanceSensor extends AbstractDistanceSensor {

    @Override
    protected Double calculateDistance(IPhysicalVehicle physicalVehicle) {
        World world = WorldModel.getInstance();
        double calculatedValue = world.getDistanceRightFrontToStreetBorder(physicalVehicle).doubleValue();
        //NormalDistribution normalDistribution = new NormalDistribution(calculatedValue, 0.01);
        return calculatedValue;
    }

}
