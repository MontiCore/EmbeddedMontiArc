package de.rwth.montisim.simulation.simulator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import de.rwth.montisim.simulation.commons.boundingbox.AABB;
import de.rwth.montisim.simulation.commons.boundingbox.BoundingBox;
import de.rwth.montisim.simulation.commons.boundingbox.CollisionTests;
import de.rwth.montisim.simulation.commons.boundingbox.OBB;
import de.rwth.montisim.simulation.commons.StaticObject;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public class CollisionDetection {
    static class MapKey {
        int gridX;
        int gridY;

        MapKey(int gridX, int gridY) {
            this.gridX = gridX;
            this.gridY = gridY;
        }

        @Override
        public boolean equals(Object o) {
            if (o == null || !(o instanceof MapKey)) return false;
            MapKey k = (MapKey) o;
            return this.gridX == k.gridX && this.gridY == k.gridY;
        }

        @Override
        public int hashCode() {
            return 997 * (gridX) ^ 991 * (gridY);
        }
    }

    static class Grid {
        static final double GRID_SIZE = 10.0;
        static final double INV_GRID_SIZE = 1.0 / GRID_SIZE;
        static final List<StaticObject> emptyList = new ArrayList<>();
        ArrayList<StaticObject> data[];
        int width;
        int height;
        int originX;
        int originY;

        void init(int width, int height, int originX, int originY) {
            this.width = width;
            this.height = height;
            this.originX = originX;
            this.originY = originY;
            data = new ArrayList[width * height];
            for (int i = 0; i < width * height; ++i) data[i] = new ArrayList<>();
        }

        void add(int x, int y, StaticObject o) {
            data[x + y * width].add(o);
        }

        List<StaticObject> get(int x, int y) {
            if (x < 0 || y < 0 || x >= width || y >= height) return emptyList;
            return data[x + y * width];
        }

        int getGridX(double xCoord) {
            return (int) Math.floor(xCoord * INV_GRID_SIZE) - originX;
        }

        int getGridY(double yCoord) {
            return (int) Math.floor(yCoord * INV_GRID_SIZE) - originY;
        }
    }

    final Vec2 halfAxesAbs = new Vec2();
    final Vec2 posXY = new Vec2();
    final HashSet<StaticObject> tested = new HashSet<>();
    final HashSet<Vehicle> testedVehicles = new HashSet<>();
    final HashMap<MapKey, List<Vehicle>> vehicleMap = new HashMap<>();
    final Grid theGrid = new Grid(); // A digital frontier
    final List<Vehicle> vehicles;
    final List<StaticObject> staticObjects;

    final Vec3 tempVec1 = new Vec3();
    final Vec3 tempVec2 = new Vec3();

    CollisionDetection(List<Vehicle> vehicles, List<StaticObject> staticObjects) {
        this.vehicles = vehicles;
        this.staticObjects = staticObjects;
    }

    void loadStaticGrid() {
        AABB worldAABB = new AABB();
        worldAABB.init();

        // Get "world AABB"
        for (StaticObject so : staticObjects) {
            if (!so.worldSpaceAABB.isPresent()) continue;
            AABB objectAABB = so.worldSpaceAABB.get();
            worldAABB.include(objectAABB.max);
            worldAABB.include(objectAABB.min);
        }

        int gridXmin = (int) Math.floor(worldAABB.min.x * Grid.INV_GRID_SIZE);
        int gridXmax = (int) Math.floor(worldAABB.max.x * Grid.INV_GRID_SIZE);
        int gridYmin = (int) Math.floor(worldAABB.min.y * Grid.INV_GRID_SIZE);
        int gridYmax = (int) Math.floor(worldAABB.max.y * Grid.INV_GRID_SIZE);

        theGrid.init(gridXmax + 1 - gridXmin, gridYmax + 1 - gridYmin, gridXmin, gridYmin);

        // Store static objects in overlapped cells
        for (StaticObject so : staticObjects) {
            if (!so.worldSpaceAABB.isPresent()) continue;
            AABB objectAABB = so.worldSpaceAABB.get();

            int objectXmin = theGrid.getGridX(objectAABB.min.x);
            int objectXmax = theGrid.getGridX(objectAABB.max.x);
            int objectYmin = theGrid.getGridY(objectAABB.min.y);
            int objectYmax = theGrid.getGridY(objectAABB.max.y);

            for (int gridX = objectXmin; gridX <= objectXmax; ++gridX) {
                for (int gridY = objectYmin; gridY <= objectYmax; ++gridY) {
                    theGrid.get(gridX, gridY).add(so);
                }
            }
        }
    }

    void checkCollisions() {
        vehicleMap.clear();
        for (Vehicle v : vehicles) {
            v.clearCollisions();
        }
        for (Vehicle v : vehicles) {
            if (!v.physicalObject.bbox.isPresent())
                throw new IllegalArgumentException("Missing BBox for vehicle " + v.physicalObject.name);
            if (!v.physicalObject.worldSpaceAABB.isPresent())
                throw new IllegalArgumentException("Missing AABB for vehicle " + v.physicalObject.name);
            BoundingBox bb = v.physicalObject.bbox.get();
            if (!(bb instanceof OBB))
                throw new IllegalArgumentException("Only OBB supported (vehicle " + v.physicalObject.name + " has BoundingBox of type " + bb.getClass().getSimpleName() + ")");
            OBB obb = (OBB) bb;
            AABB aabb = v.physicalObject.worldSpaceAABB.get();


            // Round AABB to grid coordinates
            int gridXmin = theGrid.getGridX(aabb.min.x);
            int gridXmax = theGrid.getGridX(aabb.max.x);
            int gridYmin = theGrid.getGridY(aabb.min.y);
            int gridYmax = theGrid.getGridY(aabb.max.y);

            boolean checkMultipleCells = gridXmax - gridXmin > 0 || gridYmax - gridYmin > 0;
            if (checkMultipleCells)
                tested.clear();

            // Get static collisions
            for (int gridX = gridXmin; gridX <= gridXmax; ++gridX) {
                for (int gridY = gridYmin; gridY <= gridYmax; ++gridY) {
                    for (StaticObject o : theGrid.get(gridX, gridY)) {
                        if (checkMultipleCells && tested.contains(o)) continue;
                        checkStaticCollision(v, obb, o);
                        if (checkMultipleCells)
                            tested.add(o);
                    }
                }
            }

            if (checkMultipleCells)
                testedVehicles.clear();

            // Get dynamic collisions
            for (int gridX = gridXmin; gridX <= gridXmax; ++gridX) {
                for (int gridY = gridYmin; gridY <= gridYmax; ++gridY) {
                    MapKey key = new MapKey(gridX, gridY);
                    List<Vehicle> contained = vehicleMap.get(key);
                    if (contained != null) {
                        for (Vehicle v2 : contained) {
                            if (checkMultipleCells && testedVehicles.contains(v2)) continue;
                            if (CollisionTests.collides(obb, (OBB) v2.physicalObject.bbox.get(), tempVec1, tempVec2)) {
                                v.handleVehicleCollision(v2);
                                v2.handleVehicleCollision(v);
                            }
                            if (checkMultipleCells)
                                testedVehicles.add(v2);
                        }
                    } else {
                        contained = new ArrayList<Vehicle>();
                        vehicleMap.put(key, contained);
                    }
                    contained.add(v);
                }
            }
        }
    }

    static private void checkStaticCollision(Vehicle vehicle, OBB vehicleObb, StaticObject o) {
        if (!vehicle.physicalObject.worldSpaceAABB.isPresent()) return;
        if (!o.worldSpaceAABB.isPresent()) return;
        if (!vehicle.physicalObject.worldSpaceAABB.get().overlaps3D(o.worldSpaceAABB.get())) return;

        if (o.bbox.isPresent()) {
            if (!o.bbox.get().collidesWith(vehicleObb)) return;
        }

        // Collision found
        vehicle.handleStaticCollision(o);
    }


}
