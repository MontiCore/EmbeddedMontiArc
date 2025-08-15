/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.utils;
// TODO add all missing variants & Vec2 variants & Vec4
/**
 * "In Place Math": in-memory vector and matrix operations. (Does not create new Vec/Mat objects).
 * 
 * All the functions of the form <function>To(...) take a TARGET as FIRST argument.
 * All other functions set the result to one of the arguments. (In Place)
 */
public class IPM {

    /*
        VECTOR FUNCTIONS
    */





    /*
        ADDITION
    */

    /** target = a + b */
    public static void addTo(Vec2 target, Vec2 a, Vec2 b){
        target.x = a.x + b.x;
        target.y = a.y + b.y;
    }
    /** target = a + b */
    public static void addTo(Vec3 target, Vec3 a, Vec3 b){
        target.x = a.x + b.x;
        target.y = a.y + b.y;
        target.z = a.z + b.z;
    }
    /** target = a + b */
    public static void addTo(Vec4 target, Vec4 a, Vec4 b){
        target.x = a.x + b.x;
        target.y = a.y + b.y;
        target.z = a.z + b.z;
        target.w = a.w + b.w;
    }

    /** a += b */
    public static void add(Vec2 a, Vec2 b) {
        a.x += b.x;
        a.y += b.y;
    }
    /** a += b */
    public static void add(Vec3 a, Vec3 b) {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
    }
    /** a += b */
    public static void add(Vec4 a, Vec4 b) {
        a.x += b.x;
        a.y += b.y;
        a.z += b.z;
        a.w += b.w;
    }






    /*
        SUBTRACTION
    */

    /** target = a - b */
    public static void subtractTo(Vec2 target, Vec2 a, Vec2 b){
        target.x = a.x - b.x;
        target.y = a.y - b.y;
    }
    /** target = a - b */
    public static void subtractTo(Vec3 target, Vec3 a, Vec3 b){
        target.x = a.x - b.x;
        target.y = a.y - b.y;
        target.z = a.z - b.z;
    }
    /** target = a - b */
    public static void subtractTo(Vec4 target, Vec4 a, Vec4 b){
        target.x = a.x - b.x;
        target.y = a.y - b.y;
        target.z = a.z - b.z;
        target.w = a.w - b.w;
    }
    
    /** a -= b */
    public static void subtract(Vec2 a, Vec2 b) {
        a.x -= b.x;
        a.y -= b.y;
    }
    /** a -= b */
    public static void subtract(Vec3 a, Vec3 b) {
        a.x -= b.x;
        a.y -= b.y;
        a.z -= b.z;
    }
    /** a -= b */
    public static void subtract(Vec4 a, Vec4 b) {
        a.x -= b.x;
        a.y -= b.y;
        a.z -= b.z;
        a.w -= b.w;
    }






    /*
        MULTIPLICATION
    */

    /** target = a * s */
    public static void multiplyTo(Vec2 target, Vec2 a, double s) {
        target.x = a.x * s;
        target.y = a.y * s;
    }
    /** target = a * s */
    public static void multiplyTo(Vec2 target, double s, Vec2 a) {
        target.x = a.x * s;
        target.y = a.y * s;
    }
    /** target = a * s */
    public static void multiplyTo(Vec3 target, Vec3 a, double s) {
        target.x = a.x * s;
        target.y = a.y * s;
        target.z = a.z * s;
    }
    /** target = a * s */
    public static void multiplyTo(Vec3 target, double s, Vec3 a) {
        target.x = a.x * s;
        target.y = a.y * s;
        target.z = a.z * s;
    }
    /** target = a * s */
    public static void multiplyTo(Vec4 target, Vec4 a, double s) {
        target.x = a.x * s;
        target.y = a.y * s;
        target.z = a.z * s;
        target.w = a.w * s;
    }
    /** target = a * s */
    public static void multiplyTo(Vec4 target, double s, Vec4 a) {
        target.x = a.x * s;
        target.y = a.y * s;
        target.z = a.z * s;
        target.w = a.w * s;
    }

    /** a *= s */
    public static void multiply(Vec2 a, double s) {
        a.x *= s;
        a.y *= s;
    }
    /** a *= s */
    public static void multiply(Vec3 a, double s) {
        a.x *= s;
        a.y *= s;
        a.z *= s;
    }
    /** a *= s */
    public static void multiply(Vec4 a, double s) {
        a.x *= s;
        a.y *= s;
        a.z *= s;
        a.w *= s;
    }
    
    /** Component wise multiplication. a **= b */
    public static void multiplyTo(Vec2 a, Vec2 b) {
        a.x *= b.x;
        a.y *= b.y;
    }
    /** Component wise multiplication. a **= b */
    public static void multiplyTo(Vec3 a, Vec3 b) {
        a.x *= b.x;
        a.y *= b.y;
        a.z *= b.z;
    }
    /** Component wise multiplication. a **= b */
    public static void multiplyTo(Vec4 a, Vec4 b) {
        a.x *= b.x;
        a.y *= b.y;
        a.z *= b.z;
        a.w *= b.w;
    }

    /** Component wise multiplication. target = a ** b */
    public static void multiplyTo(Vec2 target, Vec2 a, Vec2 b) {
        target.x = a.x * b.x;
        target.y = a.y * b.y;
    }
    /** Component wise multiplication. target = a ** b */
    public static void multiplyTo(Vec3 target, Vec3 a, Vec3 b) {
        target.x = a.x * b.x;
        target.y = a.y * b.y;
        target.z = a.z * b.z;
    }
    /** Component wise multiplication. target = a ** b */
    public static void multiplyTo(Vec4 target, Vec4 a, Vec4 b) {
        target.x = a.x * b.x;
        target.y = a.y * b.y;
        target.z = a.z * b.z;
        target.w = a.w * b.w;
    }






    /*
        NORMALIZATION
    */

    /** a = a/|a| */
    public static void normalize(Vec2 a) {
        double length = a.magnitude();
        if (length > 0.000001){
            double i = 1 / length;
            a.x *= i; a.y *= i;
        } else {
            a.x = 0; a.y = 0;
        }
    }
    /** a = a/|a| */
    public static void normalize(Vec3 a) {
        double length = a.magnitude();
        if (length > 0.000001){
            double i = 1 / length;
            a.x *= i; a.y *= i; a.z *= i;
        } else {
            a.x = 0; a.y = 0; a.z = 0;
        }
    }
    /** a = a/|a| */
    public static void normalize(Vec4 a) {
        double length = a.magnitude();
        if (length > 0.000001){
            double i = 1 / length;
            a.x *= i; a.y *= i; a.z *= i; a.w *= i;
        } else {
            a.x = 0; a.y = 0; a.z = 0; a.w = 0;
        }
    }
    /** target = a/|a| */
    public static void normalizeTo(Vec2 target, Vec2 a) {
        double length = a.magnitude();
        if (length > 0.000001){
            double i = 1 / length;
            target.x = a.x * i;
            target.y = a.y * i;
        } else {
            target.x = 0; target.y = 0;
        }
    }
    /** target = a/|a| */
    public static void normalizeTo(Vec3 target, Vec3 a) {
        double length = a.magnitude();
        if (length > 0.000001){
            double i = 1 / length;
            target.x = a.x * i;
            target.y = a.y * i;
            target.z = a.z * i;
        } else {
            target.x = 0; target.y = 0; target.z = 0;
        }
    }
    /** target = a/|a| */
    public static void normalizeTo(Vec4 target, Vec4 a) {
        double length = a.magnitude();
        if (length > 0.000001){
            double i = 1 / length;
            target.x = a.x * i;
            target.y = a.y * i;
            target.z = a.z * i;
            target.w = a.w * i;
        } else {
            target.x = 0; target.y = 0; target.z = 0; target.w = 0;
        }
    }






    /*
        CROSS PRODUCT
    */

    /** target = a x b */
    public static void crossTo(Vec3 target, Vec3 a, Vec3 b) {
        target.x = a.y * b.z - a.z * b.y;
        target.y = a.z * b.x - a.x * b.z;
        target.z = a.x * b.y - a.y * b.x;
    }
    /** a = a x b */
    public static void cross(Vec3 a, Vec3 b) {
        double rx = a.y * b.z - a.z * b.y;
        double ry = a.z * b.x - a.x * b.z;
        double rz = a.x * b.y - a.y * b.x;
        a.x = rx;
        a.y = ry;
        a.z = rz;
    }






    /*
        DOT PRODUCT
    */

    // Redefined as function for convenience
    /** Dot product */
    public static double dot(Vec2 a, Vec2 b) {
        return a.x * b.x + a.y * b.y;
    }
    /** Dot product */
    public static double dot(Vec3 a, Vec3 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    /** Dot product */
    public static double dot(Vec4 a, Vec4 b) {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }





    /*
        MIDPOINT
    */

    /** a = midpoint(a, b) */
    public static void midpoint(Vec2 a, Vec2 b) {
        a.x = (b.x + a.x) * 0.5;
        a.y = (b.y + a.y) * 0.5;
    }
    /** a = midpoint(a, b) */
    public static void midpoint(Vec3 a, Vec3 b) {
        a.x = (b.x + a.x) * 0.5;
        a.y = (b.y + a.y) * 0.5;
        a.z = (b.z + a.z) * 0.5;
    }
    /** a = midpoint(a, b) */
    public static void midpoint(Vec4 a, Vec4 b) {
        a.x = (b.x + a.x) * 0.5;
        a.y = (b.y + a.y) * 0.5;
        a.z = (b.z + a.z) * 0.5;
        a.w = (b.w + a.w) * 0.5;
    }
    /** target = midpoint(a, b) */
    public static void midpointTo(Vec2 target, Vec2 a, Vec2 b) {
        target.x = (b.x + a.x) * 0.5;
        target.y = (b.y + a.y) * 0.5;
    }
    /** target = midpoint(a, b) */
    public static void midpointTo(Vec3 target, Vec3 a, Vec3 b) {
        target.x = (b.x + a.x) * 0.5;
        target.y = (b.y + a.y) * 0.5;
        target.z = (b.z + a.z) * 0.5;
    }
    /** target = midpoint(a, b) */
    public static void midpointTo(Vec4 target, Vec4 a, Vec4 b) {
        target.x = (b.x + a.x) * 0.5;
        target.y = (b.y + a.y) * 0.5;
        target.z = (b.z + a.z) * 0.5;
        target.w = (b.w + a.w) * 0.5;
    }





    /*
        LINEAR INTERPOLATION
    */

    /** target = (1-alpha)*a + alpha*b */
    public static void interpolateTo(Vec2 target, Vec2 a, Vec2 b, double alpha){
        double ialpha = 1-alpha;
        target.x = ialpha*a.x + alpha*b.x;
        target.y = ialpha*a.y + alpha*b.y;
    }
    /** target = (1-alpha)*a + alpha*b */
    public static void interpolateTo(Vec3 target, Vec3 a, Vec3 b, double alpha){
        double ialpha = 1-alpha;
        target.x = ialpha*a.x + alpha*b.x;
        target.y = ialpha*a.y + alpha*b.y;
        target.z = ialpha*a.z + alpha*b.z;
    }
    /** target = (1-alpha)*a + alpha*b */
    public static void interpolateTo(Vec4 target, Vec4 a, Vec4 b, double alpha){
        double ialpha = 1-alpha;
        target.x = ialpha*a.x + alpha*b.x;
        target.y = ialpha*a.y + alpha*b.y;
        target.z = ialpha*a.z + alpha*b.z;
        target.w = ialpha*a.w + alpha*b.w;
    }





    /*
        MATRIX FUNCTIONS
    */







    /*
        Matrix vector multiplication
    */

    /** target = A*b */
    public static void multiplyTo(Vec2 target, Mat2 A, Vec2 b) {
        target.x = A.col1.x * b.x + A.col2.x * b.y;
        target.y = A.col1.y * b.x + A.col2.y * b.y;
    }
    /** target = A*b */
    public static void multiplyTo(Vec3 target, Mat3 A, Vec3 b) {
        target.x = A.col1.x * b.x + A.col2.x * b.y + A.col3.x * b.z;
        target.y = A.col1.y * b.x + A.col2.y * b.y + A.col3.y * b.z;
        target.z = A.col1.z * b.x + A.col2.z * b.y + A.col3.z * b.z;
    }
    /** target = A*b */
    public static void multiplyTo(Vec4 target, Mat4 A, Vec4 b) {
        target.x = A.col1.x * b.x + A.col2.x * b.y + A.col3.x * b.z + A.col3.x * b.w;
        target.y = A.col1.y * b.x + A.col2.y * b.y + A.col3.y * b.z + A.col3.y * b.w;
        target.z = A.col1.z * b.x + A.col2.z * b.y + A.col3.z * b.z + A.col3.z * b.w;
        target.w = A.col1.w * b.x + A.col2.w * b.y + A.col3.w * b.z + A.col3.w * b.w;
    }

    /** b = A*b */
    public static void multiply(Mat2 A, Vec2 b) {
        double rx = A.col1.x * b.x + A.col2.x * b.y;
        double ry = A.col1.y * b.x + A.col2.y * b.y;
        b.x = rx;
        b.y = ry;
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
    /** b = A*b */
    public static void multiply(Mat4 A, Vec4 b) {
        double rx = A.col1.x * b.x + A.col2.x * b.y + A.col3.x * b.z + A.col3.x * b.w;
        double ry = A.col1.y * b.x + A.col2.y * b.y + A.col3.y * b.z + A.col3.y * b.w;
        double rz = A.col1.z * b.x + A.col2.z * b.y + A.col3.z * b.z + A.col3.z * b.w;
        double rw = A.col1.w * b.x + A.col2.w * b.y + A.col3.w * b.z + A.col3.w * b.w;
        b.x = rx;
        b.y = ry;
        b.z = rz;
        b.w = rw;
    }






    /*
        Matrix-Matrix multiplication
    */

    /** target = A*B */
    public static void multiplyTo(Mat2 target, Mat2 A, Mat2 B){
        multiplyTo(target.col1, A, B.col1);
        multiplyTo(target.col2, A, B.col2);
    }
    /** target = A*B */
    public static void multiplyTo(Mat3 target, Mat3 A, Mat3 B){
        multiplyTo(target.col1, A, B.col1);
        multiplyTo(target.col2, A, B.col2);
        multiplyTo(target.col3, A, B.col3);
    }
    /** target = A*B */
    public static void multiplyTo(Mat4 target, Mat4 A, Mat4 B){
        multiplyTo(target.col1, A, B.col1);
        multiplyTo(target.col2, A, B.col2);
        multiplyTo(target.col3, A, B.col3);
        multiplyTo(target.col4, A, B.col4);
    }






    /*
        Matrix scalar multiplication
    */

    /** A *= s */
    public static void multiply(Mat2 A, double s){
        multiply(A.col1, s);
        multiply(A.col2, s);
    }
    /** A *= s */
    public static void multiply(Mat3 A, double s){
        multiply(A.col1, s);
        multiply(A.col2, s);
        multiply(A.col3, s);
    }
    /** A *= s */
    public static void multiply(Mat4 A, double s){
        multiply(A.col1, s);
        multiply(A.col2, s);
        multiply(A.col3, s);
        multiply(A.col4, s);
    }

    /** target = A * s */
    public static void multiplyTo(Mat2 target, Mat2 A, double s){
        multiplyTo(target.col1, A.col1, s);
        multiplyTo(target.col2, A.col2, s);
    }
    /** target = A * s */
    public static void multiplyTo(Mat3 target, Mat3 A, double s){
        multiplyTo(target.col1, A.col1, s);
        multiplyTo(target.col2, A.col2, s);
        multiplyTo(target.col3, A.col3, s);
    }
    /** target = A * s */
    public static void multiplyTo(Mat4 target, Mat4 A, double s){
        multiplyTo(target.col1, A.col1, s);
        multiplyTo(target.col2, A.col2, s);
        multiplyTo(target.col3, A.col3, s);
        multiplyTo(target.col4, A.col4, s);
    }






    /*
        ADDITION
    */

    /** A += B */
    public static void add(Mat2 A, Mat2 B){
        add(A.col1, B.col1);
        add(A.col2, B.col2);
    }
    /** A += B */
    public static void add(Mat3 A, Mat3 B){
        add(A.col1, B.col1);
        add(A.col2, B.col2);
        add(A.col3, B.col3);
    }
    /** A += B */
    public static void add(Mat4 A, Mat4 B){
        add(A.col1, B.col1);
        add(A.col2, B.col2);
        add(A.col3, B.col3);
        add(A.col4, B.col4);
    }

    /** target = A + B */
    public static void addTo(Mat2 target, Mat2 A, Mat2 B){
        addTo(target.col1, A.col1, B.col1);
        addTo(target.col2, A.col2, B.col2);
    }
    /** target = A + B */
    public static void addTo(Mat3 target, Mat3 A, Mat3 B){
        addTo(target.col1, A.col1, B.col1);
        addTo(target.col2, A.col2, B.col2);
        addTo(target.col3, A.col3, B.col3);
    }
    /** target = A + B */
    public static void addTo(Mat4 target, Mat4 A, Mat4 B){
        addTo(target.col1, A.col1, B.col1);
        addTo(target.col2, A.col2, B.col2);
        addTo(target.col3, A.col3, B.col3);
        addTo(target.col4, A.col4, B.col4);
    }






    /*
        SUBTRACTION
    */
    /** A += B */
    public static void subtract(Mat2 A, Mat2 B){
        subtract(A.col1, B.col1);
        subtract(A.col2, B.col2);
    }
    /** A += B */
    public static void subtract(Mat3 A, Mat3 B){
        subtract(A.col1, B.col1);
        subtract(A.col2, B.col2);
        subtract(A.col3, B.col3);
    }
    /** A += B */
    public static void subtract(Mat4 A, Mat4 B){
        subtract(A.col1, B.col1);
        subtract(A.col2, B.col2);
        subtract(A.col3, B.col3);
        subtract(A.col4, B.col4);
    }

    /** target = A + B */
    public static void subtractTo(Mat2 target, Mat2 A, Mat2 B){
        subtractTo(target.col1, A.col1, B.col1);
        subtractTo(target.col2, A.col2, B.col2);
    }
    /** target = A + B */
    public static void subtractTo(Mat3 target, Mat3 A, Mat3 B){
        subtractTo(target.col1, A.col1, B.col1);
        subtractTo(target.col2, A.col2, B.col2);
        subtractTo(target.col3, A.col3, B.col3);
    }
    /** target = A + B */
    public static void subtractTo(Mat4 target, Mat4 A, Mat4 B){
        subtractTo(target.col1, A.col1, B.col1);
        subtractTo(target.col2, A.col2, B.col2);
        subtractTo(target.col3, A.col3, B.col3);
        subtractTo(target.col4, A.col4, B.col4);
    }







    /*
        ORTHONORMALIZATION
    */

    public static void orthonormalize(Mat3 A){
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







    /*
        TRANSPOSE
    */

    /** target = transpose(A) */
    public static void transposeTo(Mat2 target, Mat2 A){
        target.col1.x = A.col1.x;
        target.col2.x = A.col1.y;
        target.col1.y = A.col2.x;
        target.col2.y = A.col2.y;
    }
    /** target = transpose(A) */
    public static void transposeTo(Mat3 target, Mat3 A){
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
    /** target = transpose(A) */
    public static void transposeTo(Mat4 target, Mat4 A){
        target.col1.x = A.col1.x;
        target.col2.x = A.col1.y;
        target.col3.x = A.col1.z;
        target.col4.x = A.col1.w;
        target.col1.y = A.col2.x;
        target.col2.y = A.col2.y;
        target.col3.y = A.col2.z;
        target.col4.y = A.col2.w;
        target.col1.z = A.col3.x;
        target.col2.z = A.col3.y;
        target.col3.z = A.col3.z;
        target.col4.z = A.col3.w;
        target.col1.w = A.col4.x;
        target.col2.w = A.col4.y;
        target.col3.w = A.col4.z;
        target.col4.w = A.col4.w;
    }

    
    /** A = transpose(A) */
    public static void transpose(Mat2 A){
        double t = A.col2.x;
        A.col2.x = A.col1.y;
        A.col1.y = t;
    }
    /** A = transpose(A) */
    public static void transpose(Mat3 A){
        double t = A.col2.x;
        A.col2.x = A.col1.y;
        A.col1.y = t;

        t = A.col3.x;
        A.col3.x = A.col1.z;
        A.col1.z = t;

        t = A.col3.y;
        A.col3.y = A.col2.z;
        A.col2.z = t;
    }
    /** A = transpose(A) */
    public static void transpose(Mat4 A){
        double t = A.col2.x;
        A.col2.x = A.col1.y;
        A.col1.y = t;

        t = A.col3.x;
        A.col3.x = A.col1.z;
        A.col1.z = t;

        t = A.col4.x;
        A.col4.x = A.col1.w;
        A.col1.w = t;

        t = A.col3.y;
        A.col3.y = A.col2.z;
        A.col2.z = t;

        t = A.col4.y;
        A.col4.y = A.col2.w;
        A.col2.w = t;

        t = A.col4.z;
        A.col4.z = A.col3.w;
        A.col3.w = t;
    }





    /*
        CROSS MATRIX
    */

    /** 
     * target = crossMatrix(v).
     * 
     * Means that a x b = crossMatrix(a)*b
     */
    public static void crossMatrixTo(Mat3 target, Vec3 v) {
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





    /*
        SCALE MATRIX
    */

    /** target = S(scale) */
    public static void scaleMatrix(Mat2 target, double scale) {
        target.col1.x = scale;
        target.col1.y = 0;
        target.col2.x = 0;
        target.col2.y = scale;
    }
    /** target = S(scale) */
    public static void scaleMatrix(Mat3 target, double scale) {
        target.col1.x = scale;
        target.col1.y = 0;
        target.col1.z = 0;
        target.col2.x = 0;
        target.col2.y = scale;
        target.col2.z = 0;
        target.col3.x = 0;
        target.col3.y = 0;
        target.col3.z = scale;
    }
    /** 
     * target = S(scale) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void homScaleMatrix(Mat3 target, double scale) {
        scaleMatrix(target, scale);
        target.col3.z = 1;
    }
    /** target = S(scale) */
    public static void scaleMatrix(Mat4 target, double scale) {
        target.col1.x = scale;
        target.col1.y = 0;
        target.col1.z = 0;
        target.col1.w = 0;
        target.col2.x = 0;
        target.col2.y = scale;
        target.col2.z = 0;
        target.col2.w = 0;
        target.col3.x = 0;
        target.col3.y = 0;
        target.col3.z = scale;
        target.col3.w = 0;
        target.col4.x = 0;
        target.col4.y = 0;
        target.col4.z = 0;
        target.col4.w = scale;
    }
    /** 
     * target = S(scale) 
     * 
     * /!\ For 3D Homogeneous coordinates.
     */
    public static void homScaleMatrix(Mat4 target, double scale) {
        scaleMatrix(target, scale);
        target.col4.w = 1;
    }






    /*
        TRANSLATION MATRIX
    */

    /** 
     * target = T(translation) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void translationMatrix(Mat3 target, Vec2 translation) {
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
     * target = T(translation) 
     * 
     * /!\ For 3D Homogeneous coordinates.
     */
    public static void translationMatrix(Mat4 target, Vec3 translation) {
        target.col1.x = 1;
        target.col1.y = 0;
        target.col1.z = 0;
        target.col1.w = 0;
        target.col2.x = 0;
        target.col2.y = 1;
        target.col2.z = 0;
        target.col2.w = 0;
        target.col3.x = 0;
        target.col3.y = 0;
        target.col3.z = 1;
        target.col3.w = 0;
        target.col4.x = translation.x;
        target.col4.y = translation.y;
        target.col4.z = translation.z;
        target.col4.w = 1;
    }




    /*
        ROTATION MATRIX
    */

    /** 
     * target = R(angle) 
     * 
     * /!\ For 2D Homogeneous coordinates.
     */
    public static void rotationMatrix(Mat3 target, double angle) {
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




    /*
        INVERSION
    */

    /** target = inv(A) */
    public static double invertTo(Mat3 target, Mat3 A){
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
