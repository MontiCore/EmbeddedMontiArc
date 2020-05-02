/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.utils;

/**
 * "In Place Math": in-memory vector and matrix operations. (Does not create new Vec/Mat objects)
 */
public class IPM {

    /*
        VECTOR FUNCTIONS
    */

    /** target = a + b */
    public static void addToVec(Vec3 a, Vec3 b, Vec3 target){
        target.x = a.x + b.x;
        target.y = a.y + b.y;
        target.z = a.z + b.z;
    }
    /** a += b */
    public static void add(Vec3 a, Vec3 b) {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
    }


    /** target = a - b */
    public static void subtractToVec(Vec3 a, Vec3 b, Vec3 target){
        target.x = a.x - b.x;
        target.y = a.y - b.y;
        target.z = a.z - b.z;
    }
    /** a -= b */
    public static void subtract(Vec3 a, Vec3 b) {
        a.x -= b.x;
        a.y -= b.y;
        a.z -= b.z;
    }


    /** target = a * s */
    public static void multiplyToVec(Vec3 a, double s, Vec3 target) {
        target.x = a.x * s;
        target.y = a.y * s;
        target.z = a.z * s;
    }
    /** a *= s */
    public static void multiply(Vec3 a, double s) {
        a.x *= s;
        a.y *= s;
        a.z *= s;
    }

    /** a = a/|a| */
    public static void normalize(Vec3 a) {
        double length = a.magnitude();
        double i = 1 / length;
        if (length > 0.000001){
            a.x *= i; a.y *= i; a.z *= i;
        } else {
            a.x = 0; a.y = 0; a.z = 0;
        }
    }
    /** target = a/|a| */
    public static void normalizeToVec(Vec3 a, Vec3 target) {
        double length = a.magnitude();
        double i = 1 / length;
        if (length > 0.000001){
            target.x = a.x * i;
            target.y = a.y * i;
            target.z = a.z * i;
        } else {
            target.x = 0; target.y = 0; target.z = 0;
        }
    }


    /** target = a x b */
    public static void crossToVec(Vec3 a, Vec3 b, Vec3 target) {
        target.x = a.y * b.z - a.z * b.y;
        target.y = a.z * b.x - a.x * b.z;
        target.z = a.x * b.y - a.y * b.x;
    }
    /** a = a x b */
    public static void crossInPlace(Vec3 a, Vec3 b) {
        double rx = a.y * b.z - a.z * b.y;
        double ry = a.z * b.x - a.x * b.z;
        double rz = a.x * b.y - a.y * b.x;
        a.x = rx;
        a.y = ry;
        a.z = rz;
    }


    // Redefined as function for convenience
    /** Dot product */
    public static double dot(Vec3 a, Vec3 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }


    /** a = midpoint(a, b) */
    public static void midpoint(Vec3 a, Vec3 b) {
        a.x = (b.x + a.x) * 0.5;
        a.y = (b.y + a.y) * 0.5;
        a.z = (b.z + a.z) * 0.5;
    }
    /** target = midpoint(a, b) */
    public static void midpointToVec(Vec3 a, Vec3 b, Vec3 target) {
        target.x = (b.x + a.x) * 0.5;
        target.y = (b.y + a.y) * 0.5;
        target.z = (b.z + a.z) * 0.5;
    }

    /*
        MATRIX FUNCTIONS
    */

    /** target = A*b */
    public static void multiplyToVec(Mat3 A, Vec3 b, Vec3 target) {
        target.x = A.col1.x * b.x + A.col2.x * b.y + A.col3.x * b.z;
        target.y = A.col1.y * b.x + A.col2.y * b.y + A.col3.y * b.z;
        target.z = A.col1.z * b.x + A.col2.z * b.y + A.col3.z * b.z;
    }

    /** b = A*b */
    public static void multiply(Mat3 A, Vec3 b) {
        double rx = A.col1.x * b.x + A.col2.x * b.y + A.col3.x * b.z;
        double ry = A.col1.y * b.x + A.col2.y * b.y + A.col3.y * b.z;
        double rz = A.col1.z * b.x + A.col2.z * b.y + A.col3.z * b.z;
        b.x = rx;
        b.y = ry;
        b.z = rz;
    }

    /** target = A*B */
    public static void multiplyToMat(Mat3 A, Mat3 B, Mat3 target){
        multiplyToVec(A, B.col1, target.col1);
        multiplyToVec(A, B.col2, target.col2);
        multiplyToVec(A, B.col3, target.col3);
    }

    /** A *= s */
    public static void multiply(Mat3 A, double s){
        A.col1.x *= s;
        A.col1.y *= s;
        A.col1.z *= s;
        A.col2.x *= s;
        A.col2.y *= s;
        A.col2.z *= s;
        A.col3.x *= s;
        A.col3.y *= s;
        A.col3.z *= s;
    }

    /** target = A * s */
    public static void multiplyToMat(Mat3 A, double s, Mat3 target){
        target.col1.x = A.col1.x * s;
        target.col1.y = A.col1.y * s;
        target.col1.z = A.col1.z * s;
        target.col2.x = A.col2.x * s;
        target.col2.y = A.col2.y * s;
        target.col2.z = A.col2.z * s;
        target.col3.x = A.col3.x * s;
        target.col3.y = A.col3.y * s;
        target.col3.z = A.col3.z * s;
    }

    /** A += B */
    public static void add(Mat3 A, Mat3 B){
        A.col1.x += B.col1.x;
        A.col1.y += B.col1.y;
        A.col1.z += B.col1.z;
        A.col2.x += B.col2.x;
        A.col2.y += B.col2.y;
        A.col2.z += B.col2.z;
        A.col3.x += B.col3.x;
        A.col3.y += B.col3.y;
        A.col3.z += B.col3.z;
    }

    /** target = A + B */
    public static void addToMat(Mat3 A, Mat3 B, Mat3 target){
        target.col1.x = A.col1.x + B.col1.x;
        target.col1.y = A.col1.y + B.col1.y;
        target.col1.z = A.col1.z + B.col1.z;
        target.col2.x = A.col2.x + B.col2.x;
        target.col2.y = A.col2.y + B.col2.y;
        target.col2.z = A.col2.z + B.col2.z;
        target.col3.x = A.col3.x + B.col3.x;
        target.col3.y = A.col3.y + B.col3.y;
        target.col3.z = A.col3.z + B.col3.z;
    }

    /** A -= B */
    public static void subtract(Mat3 A, Mat3 B){
        A.col1.x -= B.col1.x;
        A.col1.y -= B.col1.y;
        A.col1.z -= B.col1.z;
        A.col2.x -= B.col2.x;
        A.col2.y -= B.col2.y;
        A.col2.z -= B.col2.z;
        A.col3.x -= B.col3.x;
        A.col3.y -= B.col3.y;
        A.col3.z -= B.col3.z;
    }

    /** target = A - B */
    public static void subtractToMat(Mat3 A, Mat3 B, Mat3 target){
        target.col1.x = A.col1.x - B.col1.x;
        target.col1.y = A.col1.y - B.col1.y;
        target.col1.z = A.col1.z - B.col1.z;
        target.col2.x = A.col2.x - B.col2.x;
        target.col2.y = A.col2.y - B.col2.y;
        target.col2.z = A.col2.z - B.col2.z;
        target.col3.x = A.col3.x - B.col3.x;
        target.col3.y = A.col3.y - B.col3.y;
        target.col3.z = A.col3.z - B.col3.z;
    }


    public static void orthonormize(Mat3 A){
        double d1 = A.col1.dotProduct(A.col2);
        A.col2.x -= A.col1.x * d1;
        A.col2.y -= A.col1.y * d1;
        A.col2.z -= A.col1.z * d1;
        double d2 = A.col1.dotProduct(A.col3);
        A.col3.x -= A.col1.x * d2;
        A.col3.y -= A.col1.y * d2;
        A.col3.z -= A.col1.z * d2;
        double d3 = A.col2.dotProduct(A.col3);
        A.col3.x -= A.col2.x * d3;
        A.col3.y -= A.col2.y * d3;
        A.col3.z -= A.col2.z * d3;
        
        normalize(A.col1);
        normalize(A.col2);
        normalize(A.col3);
    }

    /** target = transpose(A) */
    public static void transposeToMat(Mat3 A, Mat3 target){
        target.col1.x = A.col1.x;
        target.col2.x = A.col1.y;
        target.col3.x = A.col1.z;
        target.col1.y = A.col2.x;
        target.col2.y = A.col2.y;
        target.col3.y = A.col2.z;
        target.col1.z = A.col3.x;
        target.col2.z = A.col3.y;
        target.col3.z = A.col3.z;
    }

    /** A = transpose(A) */
    public static void transpose(Mat3 A){
        double r12 = A.col2.x;
        double r13 = A.col3.x;
        double r23 = A.col3.y;
        A.col2.x = A.col1.y;
        A.col3.x = A.col1.z;
        A.col3.y = A.col2.z;
        A.col1.y = r12;
        A.col1.z = r13;
        A.col2.z = r23;
    }

    /** 
     * target = crossMatrix(v).
     * 
     * Means that a x b = crossMatrix(a)*b
     */
    public static void crossMatrixToMat(Vec3 v, Mat3 target) {
        target.col1.x = 0;
        target.col1.y = v.z;
        target.col1.z = -v.y;
        target.col2.x = -v.z;
        target.col2.y = 0;
        target.col2.z = v.x;
        target.col3.x = v.y;
        target.col3.y = -v.x;
        target.col3.z = 0;
    }

    /** 
     * target = S(scale) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void scaleMatrixToMat(double scale, Mat3 target) {
        target.col1.x = scale;
        target.col1.y = 0;
        target.col1.z = 0;
        target.col2.x = 0;
        target.col2.y = scale;
        target.col2.z = 0;
        target.col3.x = 0;
        target.col3.y = 0;
        target.col3.z = 1;
    }

    /** 
     * target = S(scale) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void scaleMatrixToMat(Vec2 scale, Mat3 target) {
        target.col1.x = scale.x;
        target.col1.y = 0;
        target.col1.z = 0;
        target.col2.x = 0;
        target.col2.y = scale.y;
        target.col2.z = 0;
        target.col3.x = 0;
        target.col3.y = 0;
        target.col3.z = 1;
    }

    /** 
     * target = T(translation) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void translationMatrixToMat(Vec2 translation, Mat3 target) {
        target.col1.x = 1;
        target.col1.y = 0;
        target.col1.z = 0;
        target.col2.x = 0;
        target.col2.y = 1;
        target.col2.z = 0;
        target.col3.x = translation.x;
        target.col3.y = translation.y;
        target.col3.z = 1;
    }

    /** 
     * target = R(angle) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void rotationMatrixToMat(double angle, Mat3 target) {
        target.col1.x = Math.cos(angle);
        target.col1.y = Math.sin(angle);
        target.col1.z = 0;
        target.col2.x = -Math.sin(angle);
        target.col2.y = Math.cos(angle);
        target.col2.z = 0;
        target.col3.x = 0;
        target.col3.y = 0;
        target.col3.z = 1;
    }

    /** target = inv(A) */
    public static double invertToMat(Mat3 A, Mat3 target){
        target.col1.x = A.col2.y*A.col3.z - A.col2.z*A.col3.y;
        target.col2.x = A.col2.z*A.col3.x - A.col2.x*A.col3.z;
        target.col3.x = A.col2.x*A.col3.y - A.col2.y*A.col3.x;

        double det = A.col1.x * target.col1.x + A.col1.y * target.col2.x + A.col1.z * target.col3.x;
        if (det < 0.0000001 && det > -0.0000001) { det = 0; }

        target.col1.y = A.col3.y * A.col1.z - A.col3.z * A.col1.y;
        target.col2.y = A.col3.z * A.col1.x - A.col3.x * A.col1.z;
        target.col3.y = A.col3.x * A.col1.y - A.col3.y * A.col1.x;

        target.col1.z = A.col1.y * A.col2.z - A.col1.z * A.col2.y;
        target.col2.z = A.col1.z * A.col2.x - A.col1.x * A.col2.z;
        target.col3.z = A.col1.x * A.col2.y - A.col1.y * A.col2.x;
        IPM.multiply(target, 1/det);
        return det;
    }
}