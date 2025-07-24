public class WeatherSensor extends AbstractSensor {
    private Double value;

    @Override
    protected void calculateValue() {
        World world = WorldModel.getInstance();
        double weatherValue = world.getWeather();
        NormalDistribution normalDistribution = new NormalDistribution(weatherValue, 0.0003);
        this.value = new Double(normalDistribution.sample());
    }

}
