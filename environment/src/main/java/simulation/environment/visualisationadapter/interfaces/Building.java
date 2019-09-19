/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.environment.visualisationadapter.interfaces;

/**
 * Created by lukas on 15.12.16.
 *
 * An interface for buildings in the environment
 * Currently unused
 *
 * Update by Florisa on 31.01.19.
 */
public interface Building extends EnvObject{
    /**
     * lists type of buildings
     */
    public enum BuildingTypes{UNIVERSITY, HOUSE, GARAGES, CHURCH}

    /**
     * @return the type of buildings
     */

    public abstract BuildingTypes getBuildingTypes();
}
