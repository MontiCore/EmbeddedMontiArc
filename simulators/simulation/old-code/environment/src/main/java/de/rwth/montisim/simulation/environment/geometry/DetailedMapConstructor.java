/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry;

import de.rwth.montisim.commons.map.IControllerNode;
import de.rwth.montisim.commons.map.PathListener;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.geometry.splines.LinearInterpolator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Created by lukas on 13.02.17.
 * Constructs the detailed Path for the controller-group for a given List of OSM-Ids
 */
@Deprecated
public class DetailedMapConstructor implements PathListener {

    private HashMap<Long, Vec3> idToPoint;

    public DetailedMapConstructor(HashMap<Long, Vec3> idToPoint) {
        this.idToPoint = idToPoint;
    }

    @Override
    /**
     * @return List of IControllerNode which specifies the detailed path
     */
    public List<IControllerNode> getDetailedPath(List<Long> list) {
        List<IControllerNode> result = new ArrayList<>();

        for (int i = 0; i < list.size() - 1; i++) {
            long osmId1 = list.get(i);
            long osmId2 = list.get(i + 1);

            ArrayList<IControllerNode> tmpList = new LinearInterpolator(idToPoint.get(osmId1), idToPoint.get(osmId2), 0.d, osmId1, osmId2, false).convertToControllerList();
            if (i != 0) {
                tmpList.remove(0);
            }
            result.addAll(tmpList);

        }
        return result;
    }
}
