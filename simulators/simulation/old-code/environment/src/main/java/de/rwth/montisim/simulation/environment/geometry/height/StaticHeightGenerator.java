/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.height;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.environment.osm.ApproximateConverter;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvBounds;

/**
 * Created by lukas on 23.02.17.
 *
 * uses fixed slopes. This functionality has to be removed from the ConcentricCircleGenerator and put in here
 */
public class StaticHeightGenerator implements HeightGenerator {

    private ConcentricCircleGenerator generator;

    public StaticHeightGenerator(EnvBounds bounds) {
        ConcentricCircleGenerator.init(bounds, true);
        this.generator = ConcentricCircleGenerator.getInstance();
    }

    @Override
    public double getGround(double x, double y) {
        return generator.getGround(x, y);
    }

    @Override
    public double[][] toHeightMap() {
        return this.generator.toHeightMap();
    }

    @Override
    public Vec2 getHeightMapMinPoint() {
        return new Vec2(0, 0);
    }

    @Override
    public Vec2 getHeightMapMaxPoint() {
        return new Vec2(2, 0);
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
