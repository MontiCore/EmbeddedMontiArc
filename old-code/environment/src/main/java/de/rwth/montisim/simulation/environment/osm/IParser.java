/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.osm;

import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.Building;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.EnvStreet;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.VisualisationEnvironmentContainer;
import de.rwth.montisim.simulation.environment.visualisationadapter.interfaces.Waterway;

import java.util.Collection;

/**
 * Created by lukas on 08.01.17.
 * An Interface which specifies the methods of an OSM-Parser
 * to remove the restriction to OSM remove getDataSet() or switch from InMemoryMapDataSet to an appropriate Interface
 */
public interface IParser {
    /**
     * parses OSM-Data
     * @throws Exception
     */
    public abstract void parse() throws Exception;


    /**
     * @return a Collection of all Streets parsed
     */
    public abstract Collection<EnvStreet> getStreets();

    /**
     * @return a collection of all Buildings parsed
     */
    public abstract Collection<Building> getBuildings();

    /**
     * @return a collection of all Buildings parsed
     */
    public abstract Collection<Waterway> getWaterways();

    /**
     * @return a collection of all Charging Stations parsed
     */
    public abstract Collection<ChargingStation> getChargingStations();


    /**
     * @return an EnvironmentContainer
     */
    public abstract VisualisationEnvironmentContainer getContainer();
}
