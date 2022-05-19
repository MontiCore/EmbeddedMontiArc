/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.environment.util;

import de.rwth.montisim.commons.controller.commons.Vertex;
import de.rwth.monticore.EmbeddedMontiArc.simulators.controller.library.structures.Graph;
import de.rwth.montisim.commons.utils.Vec3;
import org.jfree.util.Log;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.environment.object.ChargingStation;
import de.rwth.montisim.simulation.util.ServerRequest;

import java.util.Collection;

/**
 *
 */
public class ChargingStationNavigator {

    /**
     * Find nearest charging station
     *
     * @param from reference osm node
     * @return OsmID of the nearest station
     * 1 If it locates in current sector, return the OsmID of that station.
     * 2 If it locates in other sector, OsmID will be the an edge Node where the vehicle will be switched to
     * other sector by the server.
     * 3 0 if no charging station can be found.
     */
    static ChargingStation nearestCS = null;

    public static long getNearestChargingStation(String vehilcleGlobalId, long from) {
        long result = getNearestChargingStationFromServer(vehilcleGlobalId, from);
        if (result != 0) {
            return result;
        }

        // If nothing returned by the server, try to find charging station in local sector(current map)
        try {
            return getNearestChargingStationFromLocalSector(from);
        } catch (Exception e) {
            Log.warn(e);
            e.printStackTrace();
        }
        return 0;
    }

    static long getNearestChargingStationFromServer(String globalId, long from) {
        long result = 0;

        String serverHost = System.getenv("SIM_SERVER");
        String serverPort = System.getenv("SIM_PORT");
        if (serverHost == null || serverPort == null || serverHost.equals("") || serverPort.equals("")) {
            return result;
        }

        String resp = "";
        try {
            resp = ServerRequest.sendChargingStationRequest(
                    serverHost, serverPort,
                    globalId,
                    from);
        } catch (Exception e) {
            Log.warn(e);
            e.printStackTrace();
        }

        try {
            result = Long.valueOf(resp);
        } catch (NumberFormatException ignore) {
            Log.info("Unexpected server response: " + resp + ". Expect a number.");
        }
        return result;
    }

    static long getNearestChargingStationFromLocalSector(long from) throws Exception {
        Vec3 currentPos = getPositionOfOsmNode(from);
        ChargingStation nearest = null;

        try {
            // Iterate over all charging stations in current sector and find the nearest charging station
            Collection<ChargingStation> stations = World.getInstance().getContainer().getChargingStations();
            for (ChargingStation station : stations) {
                ChargingStation tmp = station;

                if (nearest == null) {
                    nearest = tmp;
                } else if (currentPos.getDistance(nearest.getLocation()) > currentPos.getDistance(tmp.getLocation())) {
                    nearest = tmp;
                }
            }
        } catch (Exception e) {
            Log.error(e);
            e.printStackTrace();
        }

        if (nearest == null) {
            nearestCS = null;
            return 0;
        }
        nearestCS = nearest;
        return getNearestOsmNodeFrom(nearest.getLocation());
    }

    public static Vec3 getPositionOfOsmNode(long osmID) throws Exception {
        Graph g = new Graph(
                World.getInstance().getControllerMap().getAdjacencies(), true);
        for (Vertex v : g.getVertices()) {
            if (v.getOsmId() == osmID) {
                return v.getPosition();
            }
        }

        throw new Exception("OsmNode " + osmID + " not found in current map");
    }

    public static long getNearestOsmNodeFrom(Vec3 realVector) throws Exception {
        long nearest = 0;
        double bestDist = Double.MAX_VALUE;
        Graph g = new Graph(
                World.getInstance().getControllerMap().getAdjacencies(), true);
        for (Vertex v : g.getVertices()) {
            double dist = v.getPosition().getDistance(realVector);
            if (dist < bestDist) {
                bestDist = dist;
                nearest = v.getOsmId();
            }
        }

        if (nearest != 0) {
            return nearest;
        }
        throw new Exception("Realvector " + realVector.toString() + " not found in current map");
    }

    public static ChargingStation getNearestCS() {
        return nearestCS;
    }
}
