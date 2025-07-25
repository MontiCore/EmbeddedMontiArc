/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.geometry;

import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.geometry.osmadapter.GeomStreet;
import de.rwth.montisim.simulation.environment.geometry.osmadapter.SplineDeterminator;
import de.rwth.montisim.simulation.environment.geometry.splines.Spline;
import de.rwth.montisim.simulation.environment.visualisationadapter.EnvNode;
import de.rwth.montisim.simulation.environment.visualisationadapter.SignTypeAndState;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by lukas on 20.03.17.
 */
public class StreetSignPositioner {

    public static void positionStreetSigns(List<GeomStreet> streets) {
        for (GeomStreet s : streets) {
            SplineDeterminator deter = s.getDeterminator();
            ArrayList<EnvNode> nodes = new ArrayList<>(s.getObject().getNodes());
            for (int i = 0; i < nodes.size(); i++) {
                EnvNode n2 = nodes.get(i);
                if (n2.getStreetSign().getType() != SignTypeAndState.EMPTY_SIGN) {
                    if (i != 0) {
                        EnvNode n1 = nodes.get(i - 1);
                        Spline spline = deter.getSplineForPoints(n1.point, n2.point);
                        if (n1.point.distance(n2.point) >= deter.getStreet().getStreetWidth().doubleValue() * 0.5) {
                            Vec3 difference = spline.getDifference().normalize();
                            Vec3 rightBorder = spline.getBorder(false, false);
                            Vec3 firstPosition = rightBorder.subtract(difference.multiply(0.5 * deter.getStreet().getStreetWidth().doubleValue()));
                            n2.getStreetSign().setOne(firstPosition);
                        } else {
                            Vec3 rightBorder = spline.getBorder(false, true);
                            n2.getStreetSign().setOne(rightBorder);
                        }
                    }

                    if (i != nodes.size() - 1) {
                        EnvNode n1 = nodes.get(i + 1);
                        Spline spline = deter.getSplineForPoints(n2.point, n1.point);
                        if (n1.point.distance(n2.point) >= deter.getStreet().getStreetWidth().doubleValue() * 0.5) {
                            Vec3 difference = spline.getDifference().normalize();
                            Vec3 leftBorder = spline.getBorder(true, true);
                            Vec3 secondPosition = leftBorder.add(difference.multiply(0.5 * deter.getStreet().getStreetWidth().doubleValue()));
                            n2.getStreetSign().setTwo(secondPosition);
                        } else {
                            Vec3 leftBorder = spline.getBorder(true, false);
                            n2.getStreetSign().setTwo(leftBorder);
                        }
                    }
                }
            }
        }

    }
}
