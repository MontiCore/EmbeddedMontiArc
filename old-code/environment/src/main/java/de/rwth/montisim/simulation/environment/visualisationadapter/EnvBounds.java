/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.visualisationadapter;

import de.rwth.montisim.commons.utils.Vec3;

/**
 * Created by lukas on 02.02.17.
 * contains the Bounds of the Environment in all directions
 */
public class EnvBounds {
    public Vec3 min;
    public Vec3 max;


    public EnvBounds(Vec3 min, Vec3 max) {
        this.min = min;
        this.max = max;
    }
}
