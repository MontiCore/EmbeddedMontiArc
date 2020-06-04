/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.utils.*;

/**
 * Class that represents a mass point of a rigid body
 */
public class Rigidbody extends DynamicObject {
    public static final double GRAVITY = -9.81;
    public Mat3 Jl = Mat3.unit(); // (local) inertia tensor
    public Mat3 J = Mat3.unit(); // (global) inertia tensor
    public Mat3 Jl_i = Mat3.unit(); // (local) inverse inertia tensor
    public Mat3 J_i = Mat3.unit(); // (global) inverse inertia tensor

    public Mat3 R_T = Mat3.unit(); // Transposed Rotation

    public Vec3 F = new Vec3(0, 0, 0); // Forces
    public Vec3 T = new Vec3(0, 0, 0); // Torque 

    // Use to compute in place -> Avoid memory trashing
    private Vec3 accel = new Vec3(0,0,0);
    private Vec3 deltaX = new Vec3(0,0,0);
    private Vec3 JW = new Vec3(0,0,0);
    private Vec3 WxJW = new Vec3(0,0,0);
    private Vec3 ang_accel = new Vec3(0,0,0);
    private Mat3 cross_matrix = new Mat3();
    private Mat3 a = new Mat3();

    public Rigidbody(String type) {
        super(type);
    }
    
    public void symplecticEuler(double delta_secs) {
        accel.set(0,0,GRAVITY);
        IPM.multiply(F, 1/mass);
        IPM.add(accel, F);
        IPM.multiply(accel, delta_secs);
        IPM.add(velocity, accel);
        
        IPM.multiplyToVec(velocity, delta_secs, deltaX);
        IPM.add(pos, deltaX);

        IPM.multiplyToVec(J, angularVelocity, JW); // JW = J*angularVelocity
        IPM.crossToVec(angularVelocity, JW, WxJW); // WxJW = angularVelocity x (J*angularVelocity)
        IPM.subtract(T, WxJW); // T = T - angularVelocity x (J*angularVelocity)
        IPM.multiplyToVec(J_i, T, ang_accel); // ang_accel = J_i * (T - angularVelocity x (J*angularVelocity))
        IPM.multiply(ang_accel, delta_secs);
        IPM.add(angularVelocity, ang_accel); // w += ang_accel * delta_secs;

        IPM.crossMatrixToMat(angularVelocity, cross_matrix); // cross_matrix = cross_matrix(angularVelocity)
        IPM.multiplyToMat(cross_matrix, rotation, a); // a = (cross_matrix(angularVelocity) * R)
        IPM.multiply(a, delta_secs); // a = (cross_matrix(angularVelocity) * rotation) * delta_secs
        IPM.add(rotation, a); // rotation += (cross_matrix(angularVelocity) * rotation) * delta_secs

        updateVars();
    }

    public void updateVars() {
        IPM.orthonormize(rotation);
        IPM.transposeToMat(rotation, R_T);

        IPM.multiplyToMat(rotation, Jl, a);
        IPM.multiplyToMat(a, R_T, J);

        IPM.multiplyToMat(rotation, Jl_i, a);
        IPM.multiplyToMat(a, R_T, J_i);
        
        F.x = 0;
        F.y = 0;
        F.z = 0;
        T.x = 0;
        T.y = 0;
        T.z = 0;
    }
}
