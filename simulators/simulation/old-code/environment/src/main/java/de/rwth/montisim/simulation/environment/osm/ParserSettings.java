/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import java.io.InputStream;
import java.util.Optional;

/**
 * Created by lukas on 16.02.17.
 *
 * A container that contains the input stream and the strategy to generate z-Coordinates
 */
public class ParserSettings {
    public enum ZCoordinates {
        ALLZERO, RANDOM, STATIC, FROM_FILE
    }

    public InputStream in;
    public ZCoordinates z;
    public String mapName = "";

    public ParserSettings(String in, ZCoordinates z) {
        this.in = getClass().getResourceAsStream(in);
        this.z = z;
    }

    @Deprecated
    public ParserSettings(InputStream in, ZCoordinates z) {
        this.in = in;
        this.z = z;
    }

    public ParserSettings(InputStream in, ZCoordinates z, String mapName) {
        this.in = in;
        this.z = z;
        this.mapName = mapName;
    }
}
