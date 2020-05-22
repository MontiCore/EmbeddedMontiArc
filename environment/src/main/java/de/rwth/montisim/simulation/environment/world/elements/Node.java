/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.world.elements;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import de.rwth.montisim.commons.utils.Vec3;

/**
 * Represents an intersection between Ways.
 * It is a virtual construct used for routing.
 */
public class Node {
    public final Vec3 point;

    // Uses the local id for road segments to easier model graphs with index based colors.
    public Vector<Integer> outRoadSegmentIDs = new Vector<>(); // Only contains road segments that can be ENTERED from the intersection (no incoming one-way roads)

    public List<Way> ways = new ArrayList<>(); // All ways passing through the intersection

    public Node(Vec3 point){
        this.point = point;
    }
}