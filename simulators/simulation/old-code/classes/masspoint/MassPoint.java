/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.vehicle.masspoint;

import de.rwth.montisim.commons.utils.Vec3;

/**
 * Class that represents a mass point of a rigid body
 */
public class MassPoint {

    /** Type of mass point */
    private MassPointType type;

    /** x_i bar of formula: Position relative to center of mass of rigid body (local coordinate system) */
    private Vec3 localPosition;

    /** r_i bar of formula: Vector pointing from center of mass of rigid body to mass point (local coordinate system) */
    private Vec3 localCenterDiff;

    /** x_i of formula: Position relative to global coordinate system */
    private Vec3 position;

    /** r_i of formula: Vector pointing from center of mass of rigid body to mass point (global coordinate system) */
    private Vec3 centerDiff;

    /** v_i of formula: Velocity relative to global coordinate system */
    private Vec3 velocity;

    /** f_i of formula: Acceleration relative to global coordinate system */
    private Vec3 force;

    /** m_i of formula: Mass of the mass point */
    private double mass;

    /** Ground Z position of mass point (for performance improvements) */
    private double groundZ;

    /** Pressure value for the mass point (e.g. tire pressure) initialised to 0.0 */
    private double pressure;

    /**
     * Constructor for a mass point that takes all variables
     * Uses deep copy of vectors to avoid that vectors can be modified externally
     *
     * @param type Type of the mass point
     * @param localPosition Position vector of mass point in local coordinate system
     * @param mass Mass of the mass point
     */
    public MassPoint(MassPointType type, Vec3 localPosition, double mass) {
        this.type = type;
        this.localPosition = localPosition.copy();
        this.localCenterDiff = new Vec3(3);
        this.position = new Vec3(3);
        this.centerDiff = new Vec3(3);
        this.velocity = new Vec3(3);
        this.force = new Vec3(3);
        this.mass = mass;
        this.groundZ = 0.0;
        this.pressure = 0.0;
    }

    /**
     * Getter for type
     * @return Type of the mass point
     */
    public MassPointType getType() {
        return type;
    }

    /**
     * Getter for local position
     * @return Deep copy of the actual vector to avoid external modifications of vector data
     */
    public Vec3 getLocalPosition() {
        return localPosition.copy();
    }

    /**
     * Getter for local center difference vector
     * @return Deep copy of the actual vector to avoid external modifications of vector data
     */
    public Vec3 getLocalCenterDiff() {
        return localCenterDiff.copy();
    }

    /**
     * Getter for global position
     * @return Deep copy of the actual vector to avoid external modifications of vector data
     */
    public Vec3 getPosition() {
        return position.copy();
    }

    /**
     * Getter for center difference vector
     * @return Deep copy of the actual vector to avoid external modifications of vector data
     */
    public Vec3 getCenterDiff() {
        return centerDiff.copy();
    }

    /**
     * Getter for velocity
     * @return Deep copy of the actual vector to avoid external modifications of vector data
     */
    public Vec3 getVelocity() {
        return velocity.copy();
    }

    /**
     * Getter for force
     * @return Deep copy of the actual vector to avoid external modifications of vector data
     */
    public Vec3 getForce() {
        return force.copy();
    }

    /**
     * Getter for mass
     * @return Mass of the mass point
     */
    public double getMass() {
        return mass;
    }

    /**
     * Setter for local position
     * @param localPosition Input vector data that is deep copied to the mass point data to avoid external modifications
     */
    public void setLocalPosition(Vec3 localPosition) {
        this.localPosition = localPosition.copy();
    }

    /**
     * Setter for local center difference vector
     * @param localCenterDiff Input vector data that is deep copied to the mass point data to avoid external modifications
     */
    public void setLocalCenterDiff(Vec3 localCenterDiff) {
        this.localCenterDiff = localCenterDiff.copy();
    }

    /**
     * Setter for position
     * @param position Input vector data that is deep copied to the mass point data to avoid external modifications
     */
    public void setPosition(Vec3 position) {
        this.position = position.copy();
    }

    /**
     * Setter for center difference vector
     * @param centerDiff Input vector data that is deep copied to the mass point data to avoid external modifications
     */
    public void setCenterDiff(Vec3 centerDiff) {
        this.centerDiff = centerDiff.copy();
    }

    /**
     * Setter for velocity
     * @param velocity Input vector data that is deep copied to the mass point data to avoid external modifications
     */
    public void setVelocity(Vec3 velocity) {
        this.velocity = velocity.copy();
    }

    /**
     * Adder for force
     * @param force Input vector that is added to the force vector
     */
    public void addForce(Vec3 force) {
        this.force = this.force.add(force);
    }

    /**
     * Resetter for force
     */
    public void resetForce() {
        this.force = new Vec3(3);
    }

    /**
     * Setter for mass
     * @param mass Mass for the mass point
     */
    public void setMass(double mass) {
        this.mass = mass;
    }

    /**
     * Getter for ground Z
     * @return Ground z
     */
    public double getGroundZ() {
        return groundZ;
    }

    /**
     * Setter for ground Z
     * @param groundZ Ground Z for the mass point
     */
    public void setGroundZ(double groundZ) {
        this.groundZ = groundZ;
    }

    /**
     * Getter for pressure
     * @return Pressure value
     */
    public double getPressure() {
        return pressure;
    }

    /**
     * Setter for pressure
     * @param pressure Pressure for the mass point
     */
    public void setPressure(double pressure) {
        this.pressure = pressure;
    }

    /**
     * Overwrite toString() to get a nice output for mass points
     * @return String that contains all information of a mass point
     */
    @Override
    public String toString() {
        return "MassPoint " + hashCode() + ": type: " + type +
                " , localPosition: " + localPosition +
                " , localCenterDiff: " + localCenterDiff +
                " , position: " + position +
                " , centerDiff: " + centerDiff +
                " , velocity: " + velocity +
                " , force: " + force +
                " , mass: " + mass +
                " , groundZ: " + groundZ +
                " , pressure: " + pressure;
    }
}
