/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.map;

import de.rwth.montisim.commons.utils.Vec2;

public interface Pathfinding {

    public Path findShortestPath(Vec2 startCoords, Vec2 targetCoords) throws Exception;
}