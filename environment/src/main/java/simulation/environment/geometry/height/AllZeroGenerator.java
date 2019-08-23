/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.geometry.height;

import javafx.geometry.Point2D;
import simulation.environment.osm.ApproximateConverter;
import simulation.environment.visualisationadapter.interfaces.EnvBounds;

/**
 * Created by lukas on 23.02.17.
 *
 * A height Generator that returns always zero
 */
public class AllZeroGenerator implements HeightGenerator{
    private EnvBounds envBounds;

    public AllZeroGenerator(EnvBounds envBounds) {
        this.envBounds = envBounds;
    }

    @Override
    public double getGround(double x, double y) {
        return 0;
    }

    @Override
    public double[][] toHeightMap() {
        return new double[0][0];
    }

    @Override
    public Point2D getHeightMapMinPoint() {
        return new Point2D(envBounds.getMinX(), envBounds.getMinY());
    }

    @Override
    public Point2D getHeightMapMaxPoint() {
        return new Point2D(envBounds.getMaxX(), envBounds.getMaxY());
    }

    @Override
    public double getHeightMapDeltaX() {
        return envBounds.getMaxX() - envBounds.getMinX();
    }

    @Override
    public double getHeightMapDeltaY() {
        return envBounds.getMaxY() - envBounds.getMinY();
    }

    @Override
    public void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        // Empty
    }
}
