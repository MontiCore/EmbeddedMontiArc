public class ObstacleSensor extends AbstractSensor {

    private List<PhysicalObject> result;
    private Object[] value;

    protected void calculateValue() {
        result.clear();
        //TODO: result.addAll Collections.synchronizedList (Alle physikalischen Objekte in einer Liste brauchen wir)
        value[0] = Double.MAX_VALUE;
        for (PhysicalObject k : result) {
            Double t = getPhysicalVehicle().getGeometryPosition().getDistance(k.getGeometryPosition());
            if (((Double) value[0]) < t) {
                value[0] = t;
                value[1] = k.getVelocity();
            }
        }
    }

}
