/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.visualisationadapter.interfaces;

import java.util.Collection;

public interface Waterway extends EnvObject {
    //TODO: This should be moved to commons, controller needs to be able to access these constant values
    /**
     * Lists Type of Streets
     */
    public enum WaterTypes {RIVER, STREAM, DITCH};

    /**
     * set STREET_WIDTH to 6 meters
     */
    public final double RIVER_WIDTH = 9;



    /**
     *
     * @return the width of this Waterway
     */
    public abstract Number getWaterwayWidth();


    /**
     *
     * @return The Type of the Street
     */
    public abstract WaterTypes getWaterType();
}
