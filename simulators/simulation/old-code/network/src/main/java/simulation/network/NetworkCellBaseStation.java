/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network;

import de.rwth.montisim.commons.simulation.IdGenerator;
import de.rwth.montisim.commons.simulation.PhysicalObject;
import de.rwth.montisim.commons.simulation.PhysicalObjectType;
import de.rwth.montisim.commons.simulation.SimulationLoopExecutable;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import de.rwth.montisim.commons.utils.Vec3;
import org.apache.commons.math3.linear.BlockRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.environment.World;
import de.rwth.montisim.simulation.util.Log;
import de.rwth.montisim.simulation.util.MathHelper;

import java.time.Duration;
import java.util.*;

/**
 * Class that represents a network cell base station in the simulation
 */
public class NetworkCellBaseStation implements SimulationLoopExecutable, PhysicalObject {

    /** Variables for the PhysicalObject interface */
    /** Position vector of the center of mass */
    private Vec3 position;
    /** Rotation matrix around the center of mass */
    private RealMatrix rotation;
    /** Velocity vector of the center of mass */
    private Vec3 velocity;
    /** Angular velocity vector around the center of mass */
    private Vec3 angularVelocity;
    /** Force vector acting on the center of mass */
    private Vec3 force;
    /** Torque vector acting around the center of mass */
    private Vec3 torque;
    /** Mass of the physical object */
    private double mass;
    /** Width of the physical object */
    private double width;
    /** Height of the physical object */
    private double height;
    /** Length of the physical object */
    private double length;
    /** Vector pointing from the center of mass position to the center of geometry position in the local coordinate system */
    private Vec3 geometryPositionOffset;
    /** Typo of the physical Object */
    private PhysicalObjectType physicalObjectType;
    /** Error flag */
    private boolean error;
    /** Collision flag */
    private boolean collision;
    /** Unique Id of the physical object */
    private long uniqueId = IdGenerator.getSharedInstance().generateUniqueId();

    /** Variables for the NetworkCellBaseStation */
    /** Set of other directly connected cell base station IDs */
    private Set<Long> connectedBaseStationIDs = Collections.synchronizedSet(new HashSet<>());

    /**
     * Constructor for a network cell base station that is standing at its position
     * Initial position at origin
     */
    public NetworkCellBaseStation() {
        position = new Vec3(new double[]{0.0, 0.0, 0.0});
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0);
        rotation = new BlockRealMatrix(rot.getMatrix());
        velocity = new Vec3(new double[]{0.0, 0.0, 0.0});
        angularVelocity = new Vec3(new double[]{0.0, 0.0, 0.0});
        force = new Vec3(new double[]{0.0, 0.0, 0.0});
        torque = new Vec3(new double[]{0.0, 0.0, 0.0});
        mass = 0.0;
        width = 5.0;
        length = 5.0;
        height = 15.0;
        geometryPositionOffset = new Vec3(new double[]{0.0, 0.0, 0.0});
        physicalObjectType = PhysicalObjectType.PHYSICAL_OBJECT_TYPE_NETWORK_CELL_BASE_STATION;
        error = false;
        collision = false;

        connectedBaseStationIDs.clear();
    }

    /**
     * Function that returns a copy of the center of mass position vector
     * @return Position vector of the center of mass
     */
    @Override
    public Vec3 getPosition() {
        return position.copy();
    }

    /**
     * Function that sets the center of mass position vector
     * @param position New position vector of the center of mass
     */
    @Override
    public void setPosition(Vec3 position) {
        this.position = position.copy();
    }

    /**
     * Function that returns a copy of the rotation matrix around the center of mass
     * @return Rotation matrix around the center of mass
     */
    @Override
    public RealMatrix getRotation() {
        return this.rotation.copy();
    }

    /**
     * Function that sets the rotation matrix around the center of mass
     * @param rotation New rotation matrix around the center of mass
     */
    @Override
    public void setRotation(RealMatrix rotation) {
        this.rotation = rotation.copy();
    }

    /**
     * Function that returns a copy of the velocity vector of the center of mass
     * @return Velocity vector of the center of mass
     */
    @Override
    public Vec3 getVelocity() {
        return this.velocity.copy();
    }

    /**
     * Function that sets the velocity vector of the center of mass
     * @param velocity New velocity vector of the center of mass
     */
    @Override
    public void setVelocity(Vec3 velocity) {
        this.velocity = velocity.copy();
    }

    /**
     * Function that returns a copy of the angular velocity vector around the center of mass
     * @return Angular velocity vector around the center of mass
     */
    public Vec3 getAngularVelocity() {
        return this.angularVelocity.copy();
    }

    /**
     * Function that sets the angular velocity vector around the center of mass
     * @param angularVelocity New angular velocity around of the center of mass
     */
    public void setAngularVelocity(Vec3 angularVelocity) {
        this.angularVelocity = angularVelocity.copy();
    }

    /**
     * Function that adds an external force acting on the center of mass
     * @param force Force vector that acts on the center of mass
     */
    @Override
    public void addForce(Vec3 force) {
        this.force = this.force.add(force);
    }

    /**
     * Function that add an external torque acting around the center of mass
     * @param torque Torque vector that acts around the center of mass
     */
    public void addTorque(Vec3 torque) {
        this.torque = this.torque.add(torque);
    }

    /**
     * Function that returns the mass of the object
     * @return Mass of the physical object
     */
    @Override
    public double getMass() {
        return this.mass;
    }

    /**
     * Function that sets the mass of the object
     * @param mass New mass of the physical object
     */
    @Override
    public void setMass(double mass) {
        this.mass = mass;
    }

    /**
     * Function that returns the width of the object
     * @return Width of the object
     */
    @Override
    public double getWidth() {
        return this.width;
    }

    /**
     * Function that sets the width of the object
     * @param width New width of the object
     */
    @Override
    public void setWidth(double width) {
        this.width = width;
    }

    /**
     * Function that returns the length of the object
     * @return Length of the object
     */
    @Override
    public double getLength() {
        return this.length;
    }

    /**
     * Function that sets the length of the object
     * @param length New length of the object
     */
    @Override
    public void setLength(double length) {
        this.length = length;
    }

    /**
     * Function that returns the height of the object
     * @return Height of the object
     */
    @Override
    public double getHeight() {
        return this.height;
    }

    /**
     * Function that sets the height of the object
     * @param height New height of the object
     */
    @Override
    public void setHeight(double height) {
        this.height = height;
    }

    /**
     * Function that returns a copy of center of geometry position vector
     * @return Position vector of the center of geometry
     */
    @Override
    public Vec3 getGeometryPosition() {
        return position.add(getGeometryPositionOffset());
    }

    /**
     * Function that sets the center of geometry position vector.
     * @param geometryPosition New position vector of the center of geometry
     */
    @Override
    public void setGeometryPosition(Vec3 geometryPosition) {
        this.position = geometryPosition.add(getGeometryPositionOffset().mapMultiply(-1.0));
    }

    /**
     * Function that returns a copy of the vector pointing from the center of mass position to the center of geometry position
     * @return Offset vector of the center of mass position to the center of geometry position
     */
    @Override
    public Vec3 getGeometryPositionOffset() {
        return this.rotation.operate(geometryPositionOffset).copy();
    }

    /**
     * Function that sets the vector pointing from the center of mass position to the center of geometry position
     * @param geometryPositionOffset New offset vector of the center of mass position to the center of geometry position
     */
    @Override
    public void setGeometryPositionOffset(Vec3 geometryPositionOffset) {
        Vec3 currentGeometryPosition = getGeometryPosition();
        RealMatrix inverseRotation = MathHelper.matrixInvert(rotation);
        this.geometryPositionOffset = inverseRotation.operate(geometryPositionOffset);
        setGeometryPosition(currentGeometryPosition);
    }

    /**
     * Function that returns the type of the physical object
     * @return Type of the physical object
     */
    @Override
    public PhysicalObjectType getPhysicalObjectType() {
        return this.physicalObjectType;
    }

    /**
     * Function that returns the error flag
     * @return Error flag of the physical object
     */
    @Override
    public boolean getError() {
        return this.error;
    }

    /**
     * Function that sets the error flag
     * @param error New Error flag of the physical object
     */
    @Override
    public void setError(boolean error) {
        Log.warning("NetworkCellBaseStation: setError - error: " + error + ", NetworkCellBaseStation at start: " + this);
        this.error = error;
        Log.warning("NetworkCellBaseStation: setError - error: " + error + ", NetworkCellBaseStation at end: " + this);
    }

    /**
     * Function that returns the collision flag
     * @return Collision flag of the physical object
     */
    @Override
    public boolean getCollision() {
        return this.collision;
    }

    /**
     * Function that sets the collision flag
     * @param collision New collision flag of the physical object
     */
    @Override
    public void setCollision(boolean collision) {
        Log.warning("NetworkCellBaseStation: setCollision - collision: " + collision + ", NetworkCellBaseStation at start: " + this);
        this.collision = collision;
        Log.warning("NetworkCellBaseStation: setCollision - collision: " + collision + ", NetworkCellBaseStation at end: " + this);
    }

    /**
     * Function that returns the id of the physical object
     * @return Id of the physical object
     */
    @Override
    public long getId() {
        return this.uniqueId;
    }

    /**
     * Function that returns a list of pairs of 3D coordinates, indicating a vector on the edges of the physical object
     * @return List of pairs of 3D points, indicating a vector on the edges of the physical object
     */
    @Override
    @Deprecated
    public List<Map.Entry<Vec3, Vec3>> getBoundaryVectors() {
        //TODO: Function is unnecessary with three dimensional collision detection
        // Build relative vectors between vertices
        Vec3 relVectorBackFront = new Vec3(new double[]{0.0, getLength(), 0.0});
        Vec3 relVectorLeftRight = new Vec3(new double[]{getWidth(), 0.0, 0.0});
        Vec3 relVectorBottomTop = new Vec3(new double[]{0.0, 0.0, getHeight()});

        // Rotate relative vectors
        relVectorBackFront = getRotation().operate(relVectorBackFront);
        relVectorLeftRight = getRotation().operate(relVectorLeftRight);
        relVectorBottomTop = getRotation().operate(relVectorBottomTop);

        // From center coordinate, compute to bottom left vertex of box
        Vec3 absBackLeft = getGeometryPosition();
        absBackLeft = absBackLeft.add(relVectorBackFront.mapMultiply(-0.5));
        absBackLeft = absBackLeft.add(relVectorLeftRight.mapMultiply(-0.5));
        absBackLeft = absBackLeft.add(relVectorBottomTop.mapMultiply(-0.5));

        // Compute absolute vectors
        Vec3 backLeft = absBackLeft.copy();
        Vec3 backRight = absBackLeft.add(relVectorLeftRight);
        Vec3 frontLeft = absBackLeft.add(relVectorBackFront);
        Vec3 frontRight = absBackLeft.add(relVectorLeftRight).add(relVectorBackFront);

        // Put vectors in list and return
        // Create map entries and insert them into list
        // Ordering is important here
        List<Map.Entry<Vec3, Vec3>> boundaryVectors = new LinkedList<>();
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(backLeft, backRight));
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(backRight, frontRight));
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontRight, frontLeft));
        boundaryVectors.add(new AbstractMap.SimpleEntry<>(frontLeft, backLeft));
        return boundaryVectors;
    }

    /**
     * Function that computes one step of the physical behaviour of the object
     * @param deltaTime Duration of the current simulation step
     */
    @Override
    public void computePhysics(Duration deltaTime) {
        //No physics computations for network cell base stations
        force = new Vec3(new double[]{0.0, 0.0, 0.0});
        torque = new Vec3(new double[]{0.0, 0.0, 0.0});
    }

    /**
     * Function that sets the position of the center of mass and the rotation of the object, in order to place the object on the surface of the world.
     * given a x, y coordinate and a z rotation
     * @param posX X component of the position of the physical object
     * @param posY Y component of the position of the physical object
     * @param rotZ Z component of the rotation of the physical object
     */
    @Override
    public void putOnSurface(double posX, double posY, double rotZ) {
        double groundZ = World.getInstance().getGround(posX, posY, this.getGeometryPosition().getEntry(2)).doubleValue();
        this.setPosition(new Vec3(new double[]{posX, posY, groundZ + 0.5 * this.getHeight()}));
        Rotation rot = new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, rotZ);
        this.setRotation(new BlockRealMatrix(rot.getMatrix()));
    }

    /**
     * Function that requests the called object to update its state for given time difference
     * @param timeDiff Difference in time measured
     */
    @Override
    public void executeLoopIteration(Duration timeDiff) {
        // Do nothing: Handled in network simulation
    }

    /**
     * Function that returns a copy of connectedBaseStationIDs
     * @return Copy of connectedBaseStationIDs
     */
    public Set<Long> getConnectedBaseStationIDs() {
        return Collections.synchronizedSet(new HashSet<>(connectedBaseStationIDs));
    }

    /**
     * Function to add a new ID to the connectedBaseStationIDs
     * @param id ID to be added
     */
    public void addConnectedBaseStationID(long id) {
        connectedBaseStationIDs.add(id);
    }

}
