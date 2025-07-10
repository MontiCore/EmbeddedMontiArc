/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry.osmadapter;

import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.geometry.splines.LinearInterpolator;
import de.rwth.montisim.simulation.environment.geometry.splines.Spline;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvBounds;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.Street;

import java.util.ArrayList;

/**
 * Created by lukas on 22.01.17.
 *
 * A SplineDeterminator for the usage of Linear interpolation
 */
public class LinearSplineDeterminator extends SplineDeterminator {

    public LinearSplineDeterminator(Street street) {
        super(street);
        initMaps(street);
        initBounds();
    }

    protected void initBounds() {
        Vec3 min = new Vec3(Double.MAX_VALUE);
        Vec3 max = new Vec3(Double.MIN_VALUE);

        for (Spline s : splines.values()) {
            ArrayList<Vec3> borderPoints = new ArrayList<>();
            LinearInterpolator leftPavement = s.getPavement(true);
            borderPoints.addAll(leftPavement.getAllBorders());
            LinearInterpolator rightPavement = s.getPavement(false);
            borderPoints.addAll(rightPavement.getAllBorders());

            for (Vec3 p : borderPoints) {
                Geometry.minimize(min, p);
                Geometry.maximize(max, p);
            }
        }

        bounds = new EnvBounds(min, max);
    }

    /**
     * @param street
     * this function constructs the splines for a given street and puts them onto the splines-map in the superclass
     */
    private void initMaps(Street street) {

        ArrayList<EnvNode> nodes = (ArrayList<EnvNode>) street.getNodes();

        for (int i = 0; i < nodes.size() - 1; i++) {
            EnvNode n1 = nodes.get(i);
            EnvNode n2 = nodes.get(i + 1);

            Vec3 p1 = n1.point.clone();
            Vec3 p2 = n2.point.clone();

            Key k = new Key(p1, p2);
            splines.put(k, new LinearInterpolator(p1, p2, Street.STREET_WIDTH, n1.osmId, n2.osmId, true));
        }
    }
}
