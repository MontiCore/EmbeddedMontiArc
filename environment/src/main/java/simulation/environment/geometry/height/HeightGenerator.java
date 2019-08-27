/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.geometry.height;

import javafx.geometry.Point2D;
import simulation.environment.osm.ApproximateConverter;

/**
 * Created by lukas on 16.02.17.
 *
 * An interface specifying a height generator
 */
public interface HeightGenerator {
    /**
     * @param x
     * @param y
     * @return the z-Coordinate corresponding to x and y
     */
    public abstract double getGround(double x, double y);

    public abstract double[][] toHeightMap();

    public abstract Point2D getHeightMapMinPoint();

    public abstract Point2D getHeightMapMaxPoint();

    public abstract double getHeightMapDeltaX();

    public abstract double getHeightMapDeltaY();

    public abstract void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter);
}
