/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import java.util.Optional;

import de.rwth.montisim.commons.boundingbox.*;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.simulation.vehicle.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.*;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties.TractionType;

public class RigidbodyPhysics implements PhysicsModel {
    // TODO model wheel friction/slipping

    private final PowerTrain powerTrain;

    public final Rigidbody rb;
    public final OBB bbox;

    // TEMP information variables
    final double wheel_radius_inv;
    final double wheel_circumference_inv;
    final double wheel_dist;
    final double wheel_width;
    final boolean wheel_contact[] = new boolean[4];
    final Vec3 wheel_pos[] = new Vec3[4];
    final Vec3 new_wheel_pos[] = new Vec3[4];

    final double accel_force[] = new double[4];
    final double brake_force[] = new double[4];

    final boolean frontAccel;
    final boolean backAccel;
    final double inv_traction_number;
    final boolean frontBrake;
    final boolean backBrake;

    // Used for computations -> Avoid memory trashing with new Vec3's, ...
    private final Vec3 contact_normal = new Vec3();
    private final Vec3 vel_change = new Vec3();
    private final Vec3 rot_change = new Vec3();
    private final Vec3 dir_l = new Vec3();
    private final Vec3 dir = new Vec3();
    private final Vec3 side_vec = new Vec3();
    private final Vec3 front_vec = new Vec3();
    private final Vec3 local_pos = new Vec3();
    private final Vec3 u = new Vec3();
    private final Vec3 ut = new Vec3();
    private final Vec3 t = new Vec3();
    private final Vec3 un = new Vec3();
    private final Vec3 u_proj = new Vec3();
    private final Vec3 delta_u = new Vec3();
    private final Vec3 impulse = new Vec3();
    private final Vec3 a = new Vec3();
    private final Vec3 p = new Vec3();
    private final Vec3 point = new Vec3();
    private final Vec3 movement_force = new Vec3();
    private final Vec3 friction_force = new Vec3();
    private final Vec3 normal_force = new Vec3();
    private final Mat3 cr_mat = new Mat3();
    private final Mat3 B = new Mat3();
    private final Mat3 A = new Mat3();
    private final Mat3 A_i = new Mat3();

    // TEMP for ground collision
    private final Vec3 ground_normal = new Vec3(0, 0, 1);
    private final Vec3 ground_pos = new Vec3(0, 0, 0);
    final Collision c = new Collision();

    public RigidbodyPhysics(Vehicle vehicle) {
        this.powerTrain = vehicle.powerTrain;

        VehicleProperties p = vehicle.properties;
        PowerTrainProperties ptp = powerTrain.properties;

        Vec3 size = new Vec3(p.body.length, p.body.width, p.body.height);
        this.bbox = new OBB();
        this.bbox.axes.setUnit();
        IPM.multiplyToVec(size, 0.5, this.bbox.half_extent);

        this.rb = new Rigidbody("RigidbodyCar");
        this.rb.bbox = Optional.of(bbox);
        this.rb.mass = p.body.mass;
        setInertiaFromBox(p.body.mass, size);
        rb.updateVars();

        // TEMP wheel info
        this.wheel_circumference_inv = 1 / (Math.PI * p.wheels.wheelDiameter);
        this.wheel_radius_inv = 2 / p.wheels.wheelDiameter;
        this.wheel_dist = size.x * 0.8;
        this.wheel_width = size.y * 0.9;

        this.frontAccel = ptp.tractionType == TractionType.FRONT
                || ptp.tractionType == TractionType.ALL;
        this.backAccel = ptp.tractionType == TractionType.REAR
                || ptp.tractionType == TractionType.ALL;
        this.inv_traction_number = 1.0 / ((frontAccel ? 2 : 0) + (backAccel ? 2 : 0));

        this.frontBrake = ptp.brakingType == TractionType.FRONT
                || ptp.brakingType == TractionType.ALL;
        this.backBrake = ptp.brakingType == TractionType.REAR
                || ptp.brakingType == TractionType.ALL;

        for (int i = 0; i < 4; ++i) {
            this.wheel_pos[i] = new Vec3();
            this.new_wheel_pos[i] = new Vec3();
        }
    }

    private void setInertiaFromBox(double mass, Vec3 size) {
        a.x = mass * (size.y * size.y + size.z * size.z) / 12;
        a.y = mass * (size.x * size.x + size.z * size.z) / 12;
        a.z = mass * (size.y * size.y + size.x * size.x) / 12;
        rb.Jl.setDiagonal(a);
        a.x = 1 / a.x;
        a.y = 1 / a.y;
        a.z = 1 / a.z;
        rb.Jl_i.setDiagonal(a);
    }

    @Override
    public void update(TimeUpdate timeUpdate) {
        // updateSteering(); -> performed in actuator
        computeStaticCollisions();
        addWheelForces(timeUpdate.deltaSeconds);
        rb.symplecticEuler(timeUpdate.deltaSeconds);
        computeEnergy(timeUpdate.deltaSeconds);
    }

    // Responsible for computing collisions between the vehicle and any static world
    // geometry.
    // Dynamic collisions are not handled physically and are only detected
    // asynchronously as "crash".
    private void computeStaticCollisions() {
        // TODO Auto-generated method stub
        temp_check_ground_collision();
    }

    private void addWheelForces(double deltaSecs) {
        // TEMP: ignore static friction -> tires have "perfect" adherence
        // double down_force = rb.mass * -Rigidbody.GRAVITY;
        // double friction = 2.0;

        // TEMP: only handle ground
        contact_normal.set(0, 0, 1);

        vel_change.set(0);
        rot_change.set(0);
        int number = 0;

        for (int i = 0; i < 4; ++i) {
            brake_force[i] = 0;
            accel_force[i] = 0;
        }

        getWheelRelPos(wheel_pos);

        for (int fb = -1; fb <= 1; fb += 2) {
            boolean front = fb == 1;
            double angle = fb == 1 ? getWheelAngle() : 0; // Angle in Radians
            dir_l.set(Math.cos(angle), Math.sin(angle), 0); // Local
            IPM.multiplyToVec(rb.rotation, dir_l, dir); // Global
            IPM.normalize(dir);

            IPM.crossToVec(contact_normal, dir, side_vec);
            if (side_vec.magnitude() < 0.01f)
                continue;
            IPM.normalize(side_vec);
            IPM.crossToVec(side_vec, contact_normal, front_vec);
            IPM.normalize(front_vec);

            for (int lr = -1; lr <= 1; lr += 2) {
                boolean left = lr == 1;
                int index = getWheelIndex(fb, lr);
                if (!wheel_contact[index])
                    continue;

                Vec3 rel_pos = wheel_pos[index];

                IPM.crossToVec(rb.angularVelocity, rel_pos, u);
                IPM.add(u, rb.velocity); // Point velocity

                // "Project" onto wheel dir (conserve length)
                IPM.multiplyToVec(side_vec, IPM.dot(u, side_vec), u_proj); // u_proj = side_vec * dot(u, side_vec)
                IPM.subtract(u_proj, u); // u_proj = (side_vec * dot(u, side_vec)) - u
                IPM.normalize(u_proj); // u_proj = normalize((side_vec * dot(u, side_vec)) - u)
                IPM.multiply(u_proj, -u.magnitude()); // u_proj = normalize(u - (side_vec * dot(u, side_vec)))*|u|
                // Desired velocity change
                IPM.subtractToVec(u_proj, u, delta_u);

                IPM.crossMatrixToMat(rel_pos, cr_mat); // cr_mat = crossMat(rel_pos)
                IPM.multiplyToMat(cr_mat, cr_mat, B); // B = crossMat(rel_pos) * crossMat(rel_pos)
                IPM.multiplyToMat(rb.J_i, rb.mass, A);
                IPM.multiplyToMat(A, B, cr_mat); // cr_mat = rb.mass * rb.J_i * crossMat(rel_pos) * crossMat(rel_pos)
                A.setUnit();
                IPM.subtract(A, cr_mat); // A = diag(1) - rb.mass *(rb.J_i * crossMat(rel_pos) * crossMat(rel_pos))
                double det = IPM.invertToMat(A, A_i);
                if (det != 0) {
                    IPM.multiplyToVec(A_i, delta_u, impulse);

                    IPM.add(vel_change, impulse);

                    IPM.multiply(impulse, rb.mass);
                    IPM.crossToVec(rel_pos, impulse, a);
                    IPM.multiply(rb.J_i, a);
                    IPM.add(rot_change, a);
                    number++;
                }

                double wheel_force = getWheelForce(IPM.dot(u, side_vec), IPM.dot(u, front_vec), deltaSecs, front,
                        index);
                IPM.multiplyToVec(front_vec, wheel_force, movement_force);
                IPM.add(rb.F, movement_force);
                IPM.crossToVec(rel_pos, movement_force, a);
                IPM.add(rb.T, a);
            }
        }

        // Apply "wheel track" impulses + "correct velocity" to "conserve momentum"
        // (rough approximation)
        if (number > 0) {
            IPM.multiply(vel_change, 1.0 / number);
            IPM.multiply(rot_change, 1.0 / number);
            double l = rb.velocity.magnitude();
            IPM.add(rb.velocity, vel_change);
            IPM.normalize(rb.velocity);
            IPM.multiply(rb.velocity, l);
            IPM.add(rb.angularVelocity, rot_change);
        }

        // Transform wheel pos from relative to absolute
        for (Vec3 p : wheel_pos) {
            IPM.add(p, rb.pos);
        }
    }

    private void getWheelRelPos(Vec3 relPos[]) {
        for (int fb = -1; fb <= 1; fb += 2) {
            double x = wheel_dist * 0.5 * fb;
            for (int lr = -1; lr <= 1; lr += 2) {
                int index = getWheelIndex(fb, lr);
                double y = wheel_width * 0.5f * lr;
                local_pos.set(x, y, -bbox.half_extent.z);
                IPM.multiplyToVec(rb.rotation, local_pos, relPos[index]);
            }
        }
    }

    private int getWheelIndex(int fb, int lr) {
        return ((fb + 1) >> 1) + (((lr + 1) >> 1) << 1);
    }

    private int getWheelIndex(boolean front, boolean left) {
        return (front ? 1 : 0) + (left ? 2 : 0);
    }

    /**
     * @return The steering angle in RADIANS
     */
    private double getWheelAngle() {
        return (Double) powerTrain.steeringValue.value() * Geometry.DEG_TO_RAD;
    }

    private double getWheelForce(double wheelSpeed, double front_vel, double deltaSecs, boolean front, int index) {
        double f_accel = 0;
        double f_brake = 0;
        if ((front && frontAccel) || (!front && backAccel)) {
            // TODO right now this assumes 1:1 coupling between motor & wheels
            double ratio = powerTrain.getTransmissionRatio();
            double rpm = wheelSpeed * wheel_circumference_inv * 60 * ratio;
            double i_accel = (Double) powerTrain.gasValue.value();
            // TODO right now this assumes 1:1 coupling between motor & wheels
            f_accel = i_accel * powerTrain.motor.getMaxTorque(rpm) * wheel_radius_inv * inv_traction_number * ratio;
        }

        if ((front && frontBrake) || (!front && backBrake)) {
            double i_brake = (Double) powerTrain.brakingValue.value();
            double f_brake_abs = i_brake * powerTrain.properties.maxBrakingForce;
            // Limit braking force to avoid integration overshooting (when braking at near 0
            // speed)
            double max_f = Math.abs(front_vel) * rb.mass / deltaSecs;
            if (f_brake_abs > max_f)
                f_brake_abs = max_f;
            f_brake = -Math.signum(front_vel) * f_brake_abs;
        }

        brake_force[index] = f_brake;
        accel_force[index] = f_accel;

        return f_accel + f_brake;
    }

    /**
     * Computes an estimation of the energy used by the motor for acceleration and
     * the energy output from braking.
     */
    private void computeEnergy(double delta_t) {
        // Get wheel positions after integration.
        getWheelRelPos(new_wheel_pos);
        double brakeEnergy = 0;
        double accelEnergy = 0;
        for (int i = 0; i < 4; i++) {
            IPM.add(new_wheel_pos[i], rb.pos);
            IPM.subtract(new_wheel_pos[i], wheel_pos[i]); // new_wheel_pos[i] = delta x
            double deltaX = new_wheel_pos[i].magnitude();
            brakeEnergy += deltaX * brake_force[i];
            accelEnergy += deltaX * accel_force[i];
        }
        if (accelEnergy > 0.01)
            powerTrain.motor.consume(accelEnergy, delta_t);
        if (brakeEnergy > 0.01)
            powerTrain.motor.regenerate(brakeEnergy, delta_t);
    }

    /**
     * A simplified collision detection that only checks collision with the ground
     * plane (ground_normal & ground_pos)
     */
    private void temp_check_ground_collision() {
        for (int i = 0; i < 4; ++i)
            wheel_contact[i] = false;
        c.normal.set(ground_normal);
        double ref = IPM.dot(ground_normal, ground_pos);
        for (int dx = -1; dx <= 1; dx += 2)
            for (int dy = -1; dy <= 1; dy += 2)
                for (int dz = -1; dz <= 1; dz += 2) {
                    IPM.multiplyToVec(bbox.axes.col1, bbox.half_extent.x * dx, point);
                    IPM.multiplyToVec(bbox.axes.col2, bbox.half_extent.y * dy, a);
                    IPM.add(point, a);
                    IPM.multiplyToVec(bbox.axes.col3, bbox.half_extent.z * dz, a);
                    IPM.add(point, a);
                    IPM.multiplyToVec(rb.rotation, point, c.rel_pos);
                    IPM.addToVec(c.rel_pos, rb.pos, point);
                    c.penetration = IPM.dot(point, ground_normal) - ref;
                    if (c.penetration < 0) {
                        temp_compute_collision_reaction(c, dz == -1);
                    }
                    // Hack to check wheel/ground contact + no details -> assume contact normal is
                    // ground normal
                    if (dz == -1) {
                        // Use 10cm to ""emulate"" suspensions
                        if (c.penetration < 0.1) {
                            wheel_contact[getWheelIndex(dx, dy)] = true;
                        }
                    }
                }
    }

    private void temp_compute_collision_reaction(Collision c, boolean is_wheel) {
        double e = 0.5; // Restitution?
        double friction = 0.9;
        double stiffness = 100;

        IPM.crossToVec(rb.angularVelocity, c.rel_pos, u);
        IPM.add(u, rb.velocity); // Point velocity
        double u_normal = IPM.dot(u, c.normal);
        IPM.multiplyToVec(c.normal, u_normal, un);

        IPM.subtractToVec(u, un, ut); // Tangential vel
        t.set(0); // Normalized Tangent dir
        double tl = ut.magnitude();
        if (tl > 0.0001)
            IPM.multiplyToVec(ut, 1 / tl, t);

        // Impulse
        IPM.crossToVec(c.rel_pos, c.normal, a);
        IPM.multiply(rb.J_i, a);
        IPM.crossInPlace(a, c.rel_pos);
        // a = cross(rb.J_i * cross(c.rel_pos, c.normal), c.rel_pos);
        double denom = 1 / rb.mass + IPM.dot(c.normal, a);
        IPM.multiplyToVec(un, (-1.0 - e) / denom, p);
        /*
         * auto friction_impulse = t * (p.length() * -friction); p += friction_impulse;
         */
        if (IPM.dot(p, c.normal) > 0) {
            IPM.multiplyToVec(p, (1 / rb.mass), a);
            IPM.add(rb.velocity, a);
            IPM.crossToVec(c.rel_pos, p, a);
            IPM.multiply(rb.J_i, a);
            IPM.add(rb.angularVelocity, a);
        }

        // Penalty forces
        IPM.multiplyToVec(c.normal, stiffness * -c.penetration * rb.mass, normal_force);
        IPM.add(rb.F, normal_force);
        IPM.crossToVec(c.rel_pos, normal_force, a);
        IPM.add(rb.T, a);
        if (!is_wheel) {
            IPM.multiplyToVec(t, normal_force.magnitude() * -friction, friction_force);
            IPM.add(rb.F, friction_force);
            IPM.crossToVec(c.rel_pos, friction_force, a);
            IPM.add(rb.T, a);
        }
    }

    @Override
    public DynamicObject getPhysicalObject() {
        return rb;
    }

    @Override
    public void setGroundPosition(Vec3 pos, Vec2 front) {
        rb.pos.set(pos);
        rb.pos.z += bbox.half_extent.z;
        // Create rotation matrix from vectors
        front_vec.set(front, 0);
        IPM.normalize(front_vec);
        a.set(0, 0, 1);
        IPM.crossToVec(a, front_vec, side_vec);
        rb.rotation.col1.set(front_vec);
        rb.rotation.col2.set(side_vec);
        rb.rotation.col3.set(a);
        rb.updateVars();
    }

}