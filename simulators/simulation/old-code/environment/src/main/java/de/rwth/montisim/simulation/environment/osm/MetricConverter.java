/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import de.rwth.montisim.commons.utils.Vec2;

/**
 * Created by lukas on 24.01.17.
 * Specifies the Methods of a Converter from longitude/latitude to metric x and y
 */
public interface MetricConverter {

    /// Converts Geographic Coordinates to Meters
    public abstract Vec2 coordinatesToMeters(Coordinates coordinates);

    /// Converts Meters to Geographic Coordinates
    public abstract Coordinates metersToCoordinates(Vec2 position);
}
