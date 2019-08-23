/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.map;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.utils.Point3D;

/**
 * Created by lukas on 31.01.17.
 */
public interface IControllerNode {
    public final long INTERPOLATED_NODE = -1;

    public final double INTERPOLATION_DISTANCE = 2;

    public abstract Point3D getPoint();

    public abstract long getId();

    public abstract long getOsmId();
}
