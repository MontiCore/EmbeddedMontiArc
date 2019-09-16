/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.geometry.height;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point2D;
import simulation.environment.osm.ApproximateConverter;
import simulation.environment.visualisationadapter.interfaces.EnvBounds;

/**
 * Created by lukas on 23.02.17.
 *
 * uses fixed slopes. This functionality has to be removed from the ConcentricCircleGenerator and put in here
 */
public class StaticHeightGenerator implements HeightGenerator {

    private ConcentricCircleGenerator generator;

    public StaticHeightGenerator(EnvBounds bounds) {
        ConcentricCircleGenerator.init(bounds,true);
        this.generator = ConcentricCircleGenerator.getInstance();
    }

    @Override
    public double getGround(double x, double y) {
        return generator.getGround(x,y);
    }

    @Override
    public double[][] toHeightMap() {
        return this.generator.toHeightMap();
    }

    @Override
    public Point2D getHeightMapMinPoint() {
        return new Point2D(0,0);
    }

    @Override
    public Point2D getHeightMapMaxPoint() {
        return new Point2D(2,0);
    }

    @Override
    public double getHeightMapDeltaX() {
        return 1.0;
    }

    @Override
    public double getHeightMapDeltaY() {
        return 1.0;
    }

    @Override
    public void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        // Empty
    }
}
