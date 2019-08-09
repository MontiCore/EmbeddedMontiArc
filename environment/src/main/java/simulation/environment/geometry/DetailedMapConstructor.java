/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.environment.geometry;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.IControllerNode;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map.PathListener;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;
import simulation.environment.geometry.splines.LinearInterpolator;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Created by lukas on 13.02.17.
 * Constructs the detailed Path for the controller-group for a given List of OSM-Ids
 */
@Deprecated
public class DetailedMapConstructor implements PathListener {

    private HashMap<Long, Point3D> idToPoint;

    public DetailedMapConstructor(HashMap<Long, Point3D> idToPoint) {
        this.idToPoint = idToPoint;
    }

    @Override
    /**
     * @return List of IControllerNode which specifies the detailed path
     */
    public List<IControllerNode> getDetailedPath(List<Long> list) {
        List<IControllerNode> result = new ArrayList<>();

        for(int i = 0; i < list.size() - 1; i++) {
            long osmId1 = list.get(i);
            long osmId2 = list.get(i + 1);

            ArrayList<IControllerNode> tmpList = new LinearInterpolator(idToPoint.get(osmId1), idToPoint.get(osmId2), 0.d, osmId1, osmId2, false).convertToControllerList();
            if(i != 0) {
                tmpList.remove(0);
            }
            result.addAll(tmpList);

        }
        return result;
    }
}