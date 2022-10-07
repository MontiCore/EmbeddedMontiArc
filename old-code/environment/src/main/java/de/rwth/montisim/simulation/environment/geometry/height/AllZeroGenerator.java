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
 * A height Generator that returns always zero
 */
public class AllZeroGenerator implements HeightGenerator {
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
    public Vec2 getHeightMapMinPoint() {
        return envBounds.min.asVec2();
    }

    @Override
    public Vec2 getHeightMapMaxPoint() {
        return envBounds.max.asVec2();
    }

    @Override
    public double getHeightMapDeltaX() {
        return envBounds.max.x - envBounds.min.x;
    }

    @Override
    public double getHeightMapDeltaY() {
        return envBounds.max.y - envBounds.min.y;
    }

    @Override
    public void setLongLatToMetersConverter(ApproximateConverter longLatToMeterConverter) {
        // Empty
    }
}
