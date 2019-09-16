/* (c) https://github.com/MontiCore/monticore */
package simulation.environment.visualisationadapter.interfaces;

import java.util.List;

/**
 * Created by Danilo 07.02.2019.
 *
 * An interface for all Objects in the environment
 */
public interface EnvObject {
    /**
     * @return a List of the Nodes this object consists of
     */
    public abstract List<EnvNode> getNodes();

    /**
     * @return a Tag containing more information on this Object
     */
    public abstract EnvTag getTag();

    /**
     * @return the OpenStreetMap of this Object
     */
    public abstract long getOsmId();
}
