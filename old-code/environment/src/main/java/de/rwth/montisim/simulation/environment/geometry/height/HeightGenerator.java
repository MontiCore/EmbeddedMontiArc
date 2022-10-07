/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.height;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.environment.osm.ApproximateConverter;

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

    public abstract Vec2 getHeightMapMinPoint();

    public abstract Vec2 getHeightMapMaxPoint();

    public abstract double getHeightMapDeltaX();

    public abstract double getHeightMapDeltaY();

    public abstract void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter);
}
