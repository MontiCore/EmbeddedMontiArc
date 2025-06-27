/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.map;

import java.util.List;

/**
 * Created by lukas on 13.02.17.
 */
public interface PathListener {
    /**
     * @param osmIds
     * @return a detailed version of the path for a list of given osm-ids
     */
    public abstract List<ControllerNode> getDetailedPath(List<Long> osmIds);
}
