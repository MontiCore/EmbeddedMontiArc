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
package simulation.environment.util;

import commons.controller.commons.Vertex;
import org.apache.commons.math3.linear.RealVector;
import org.jfree.util.Log;
import simulation.environment.WorldModel;
import simulation.environment.object.ChargingStation;
import simulation.util.ServerRequest;

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
     * 1. If it locates in current sector, return the OsmID of that station.
     * 2. If it locates in other sector, OsmID will be the an edge Node where the vehicle will be switched to
     * other sector by the server.
     * 3. 0 if no charging station can be found.
     */

    static ChargingStation nearestCS = null;

    public static long getNearestChargingStation(long from) {
        long result = getNearestChargingStationFromServer(from);
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

    static long getNearestChargingStationFromServer(long from) {
        ChargingStation station = null;

        String serverHost = System.getenv("SIM_SERVER");
        String serverPort = System.getenv("SIM_PORT");
        if (serverHost.equals("") || serverPort.equals("")) {
            return 0;
        }

        String resp = "";
        try {
            resp = ServerRequest.sendChargingStationRequest(
                    "http://" + serverHost + ":" + serverPort,
                    WorldModel.getInstance().getParser().getMapName(),
                    from);
        } catch (Exception e) {
            Log.warn(e);
            e.printStackTrace();
        }

        if (resp.equals("")) {
            return 0;
        } else {
            // TODO validate format of resp
            return Long.valueOf(resp);
        }
    }

    static long getNearestChargingStationFromLocalSector(long from) throws Exception {
        RealVector currentPos = getPositionOfOsmNode(from);
        ChargingStation nearest = null;

        try {
            // Iterate over all charging stations in current sector and find the nearest charging station
            Collection<ChargingStation> stations = WorldModel.getInstance().getParser().getChargingStations();
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
        return nearest.getOsmId();
    }

    private static RealVector getPositionOfOsmNode(long osmID) throws Exception {
        structures.Graph g = new structures.Graph(
                WorldModel.getInstance().getControllerMap().getAdjacencies(), true);
        for (Vertex v : g.getVertices()) {
            if (v.getOsmId() == osmID) {
                return v.getPosition();
            }
        }

        throw new Exception("OsmNode " + osmID + " not found in current map");
    }

    public static ChargingStation getNearestCS(){
        return nearestCS;
    }
}
