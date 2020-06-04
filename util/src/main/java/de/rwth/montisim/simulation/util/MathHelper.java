/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.util;

// import org.apache.commons.math3.geometry.euclidean.twod.Euclidean2D;
// import org.apache.commons.math3.geometry.euclidean.twod.PolygonsSet;
// import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
// import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
// import org.apache.commons.math3.geometry.partitioning.Region;
// import org.apache.commons.math3.geometry.partitioning.RegionFactory;
// import org.apache.commons.math3.random.RandomDataGenerator;
// import java.util.LinkedList;
// import java.util.List;
// import java.util.Map;
import org.apache.commons.math3.linear.*;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Mat4;


/**
 * Class that provides static access to common math operations
 */
public final class MathHelper {


    /**
     * Function that computes the inverse of an input matrix, if possible
     * Perform some simple checks before actually computing the matrix inverse
     *
     * @param matrix Matrix that is used for the inverse computation, should be square and invertible matrix
     * @return Inverse matrix of the input matrix
     */
    private static RealMatrix matrixInvert(RealMatrix matrix) {
        // Check for valid input
        if(!matrix.isSquare()){
            throw new IllegalArgumentException("Matrix " + matrix + "should be a square matrix.");
        }

        if (matrix.getColumnDimension() > 0) {
            LUDecomposition lu = new LUDecomposition(matrix);

            if (lu.getDeterminant() != 0.0) {
                DecompositionSolver solver = lu.getSolver();
                return solver.getInverse();
            }
        }

        throw new IllegalArgumentException("Matrix " + matrix + "should not be singular.");
    }

    public static Mat3 matrixInvert(Mat3 matrix){
        RealMatrix mIn = new Array2DRowRealMatrix(3,3);
        mIn.setEntry(0, 0, matrix.col1.x);
        mIn.setEntry(1, 0, matrix.col1.y);
        mIn.setEntry(2, 0, matrix.col1.z);
        mIn.setEntry(0, 1, matrix.col2.x);
        mIn.setEntry(1, 1, matrix.col2.y);
        mIn.setEntry(2, 1, matrix.col2.z);
        mIn.setEntry(0, 2, matrix.col3.x);
        mIn.setEntry(1, 2, matrix.col3.y);
        mIn.setEntry(2, 2, matrix.col3.z);
        RealMatrix mOut = matrixInvert(mIn);
        Mat3 res = new Mat3();
        res.col1.x = mOut.getEntry(0, 0);
        res.col1.y = mOut.getEntry(1, 0);
        res.col1.z = mOut.getEntry(2, 0);
        res.col2.x = mOut.getEntry(0, 1);
        res.col2.y = mOut.getEntry(1, 1);
        res.col2.z = mOut.getEntry(2, 1);
        res.col3.x = mOut.getEntry(0, 2);
        res.col3.y = mOut.getEntry(1, 2);
        res.col3.z = mOut.getEntry(2, 2);
        return res;
    }

    public static Mat4 matrixInvert(Mat4 matrix){
        RealMatrix mIn = new Array2DRowRealMatrix(4,4);
        mIn.setEntry(0, 0, matrix.col1.x);
        mIn.setEntry(1, 0, matrix.col1.y);
        mIn.setEntry(2, 0, matrix.col1.z);
        mIn.setEntry(3, 0, matrix.col1.w);
        mIn.setEntry(0, 1, matrix.col2.x);
        mIn.setEntry(1, 1, matrix.col2.y);
        mIn.setEntry(2, 1, matrix.col2.z);
        mIn.setEntry(3, 1, matrix.col2.w);
        mIn.setEntry(0, 2, matrix.col3.x);
        mIn.setEntry(1, 2, matrix.col3.y);
        mIn.setEntry(2, 2, matrix.col3.z);
        mIn.setEntry(3, 2, matrix.col3.w);
        mIn.setEntry(0, 3, matrix.col4.x);
        mIn.setEntry(1, 3, matrix.col4.y);
        mIn.setEntry(2, 3, matrix.col4.z);
        mIn.setEntry(3, 3, matrix.col4.w);
        RealMatrix mOut = matrixInvert(mIn);
        Mat4 res = new Mat4();
        res.col1.x = mOut.getEntry(0, 0);
        res.col1.y = mOut.getEntry(1, 0);
        res.col1.z = mOut.getEntry(2, 0);
        res.col1.w = mOut.getEntry(3, 0);
        res.col2.x = mOut.getEntry(0, 1);
        res.col2.y = mOut.getEntry(1, 1);
        res.col2.z = mOut.getEntry(2, 1);
        res.col2.w = mOut.getEntry(3, 1);
        res.col3.x = mOut.getEntry(0, 2);
        res.col3.y = mOut.getEntry(1, 2);
        res.col3.z = mOut.getEntry(2, 2);
        res.col3.w = mOut.getEntry(3, 2);
        res.col4.x = mOut.getEntry(0, 3);
        res.col4.y = mOut.getEntry(1, 3);
        res.col4.z = mOut.getEntry(2, 3);
        res.col4.w = mOut.getEntry(3, 3);
        return res;
    }

    /**
     * Function that performs a re-orthonormalization for a given rotation matrix
     *
     * @param matrix Input 3x3 rotation matrix that is used in re-orthonormalization
     * @return Result 3x3 rotation matrix that is now orthonormal again
     */
    // public static RealMatrix matrixReOrthonormalize(RealMatrix matrix) {
    //     // Checks for valid input
    //     if (matrix.getColumnDimension() != 3 || !matrix.isSquare()) {
    //         throw new IllegalArgumentException("Matrix " + matrix + " should be a 3x3 matrix.");
    //     }
    //     if (matrix.getColumnVector(0).getNorm() == 0 || matrix.getColumnVector(1).getNorm() == 0 || matrix.getColumnVector(2).getNorm() == 0){
    //         throw new IllegalArgumentException("Matrix " + matrix + " should have no zero-norm columns.");
    //     }

    //     // Prepare variables

    //     // Approach: Gram Schmidt re-orthonormalization
    //     Vec3 v0 = matrix.getColumnVector(0);
    //     Vec3 v1 = matrix.getColumnVector(1);
    //     Vec3 v2 = matrix.getColumnVector(2);

    //     v0 = v0.mapDivide(v0.getNorm());

    //     v1 = v1.subtract(v0.mapMultiply(v0.dotProduct(v1)));
    //     v1 = v1.mapDivide(v1.getNorm());

    //     v2 = v2.subtract(v0.mapMultiply(v0.dotProduct(v2))).subtract(v1.mapMultiply(v1.dotProduct(v2)));
    //     v2 = v2.mapDivide(v2.getNorm());

    //     matrix.setColumnVector(0, v0);
    //     matrix.setColumnVector(1, v1);
    //     matrix.setColumnVector(2, v2);
    //     return matrix;
    // }

    /**
     * Function that converts a vector to a matrix such that the matrix multiplied with another vector is the cross product
     * Perform some simple checks before actually computing the matrix
     * Only supports 3-dimensional vectors and 3x3 matrices as result
     *
     * @param vector 3-dimensional vector for which the corresponding cross product matrix should be computed
     * @return 3x3 cross product matrix for the input 3-dimensional vector
     */
    // public static RealMatrix vectorToCrossProductMatrix(Vec3 vector) {
    //     // Check for valid input
    //     if(vector.getDimension() != 3){
    //         throw new IllegalArgumentException("Vector " + vector + " should be a three dimensional vector.");
    //     }

    //     double[][] matrixEntries = {{0.0, -vector.getEntry(2), vector.getEntry(1)}, {vector.getEntry(2), 0.0, -vector.getEntry(0)}, {-vector.getEntry(1), vector.getEntry(0), 0.0}};
    //     return new BlockRealMatrix(matrixEntries);
    // }


    /**
     * Function that checks if two oriented bounding boxes intersect
     *
     * @param boxOne First bounding box
     * @param boxTwo Second bounding box
     * @return True if the boxes intersect, otherwise false
     */
    // public static boolean checkIntersection(OrientedBoundingBox boxOne, OrientedBoundingBox boxTwo) {
    //     // Using separating axis theorem the bounding boxes intersect iff no separating plane exists for these 15 cases
    //     return !(existsSeparatingPlane(boxOne.getLocalAxisX(), boxOne, boxTwo) ||
    //             existsSeparatingPlane(boxOne.getLocalAxisY(), boxOne, boxTwo) ||
    //             existsSeparatingPlane(boxOne.getLocalAxisZ(), boxOne, boxTwo) ||
    //             existsSeparatingPlane(boxTwo.getLocalAxisX(), boxOne, boxTwo) ||
    //             existsSeparatingPlane(boxTwo.getLocalAxisY(), boxOne, boxTwo) ||
    //             existsSeparatingPlane(boxTwo.getLocalAxisZ(), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisX(), boxTwo.getLocalAxisX()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisX(), boxTwo.getLocalAxisY()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisX(), boxTwo.getLocalAxisZ()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisY(), boxTwo.getLocalAxisX()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisY(), boxTwo.getLocalAxisY()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisY(), boxTwo.getLocalAxisZ()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisZ(), boxTwo.getLocalAxisX()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisZ(), boxTwo.getLocalAxisY()), boxOne, boxTwo) ||
    //             existsSeparatingPlane(crossProduct(boxOne.getLocalAxisZ(), boxTwo.getLocalAxisZ()), boxOne, boxTwo));
    // }

    // /**
    //  * Function that checks whether there exists a separating plane between 2 oriented bounding boxes
    //  *
    //  * @param separatingAxis Vector to which the separating plane should be orthogonal to
    //  * @param boxOne First oriented bounding box
    //  * @param boxTwo Second oriented bounding box
    //  * @return True if separating plane exists, otherwise false
    //  */
    // private static boolean existsSeparatingPlane(Vec3 separatingAxis, OrientedBoundingBox boxOne, OrientedBoundingBox boxTwo) {
    //     Vec3 posDifference = boxTwo.getPosition().subtract(boxOne.getPosition());
    //     return Math.abs(posDifference.dotProduct(separatingAxis)) >
    //             (Math.abs(boxOne.getLocalAxisX().mapMultiply(boxOne.getHalfWidth()).dotProduct(separatingAxis)) +
    //                     Math.abs(boxOne.getLocalAxisY().mapMultiply(boxOne.getHalfLength()).dotProduct(separatingAxis)) +
    //                     Math.abs(boxOne.getLocalAxisZ().mapMultiply(boxOne.getHalfHeight()).dotProduct(separatingAxis)) +
    //                     Math.abs(boxTwo.getLocalAxisX().mapMultiply(boxTwo.getHalfWidth()).dotProduct(separatingAxis)) +
    //                     Math.abs(boxTwo.getLocalAxisY().mapMultiply(boxTwo.getHalfLength()).dotProduct(separatingAxis)) +
    //                     Math.abs(boxTwo.getLocalAxisZ().mapMultiply(boxTwo.getHalfHeight()).dotProduct(separatingAxis)));
    // }

    /**
     * Function that checks if two 2D spaces defined by real vectors have a non empty 2D space intersection
     *
     * @param vectorsOne First pairs of vector start points and end points that create a 2D space, ordered such that left halves of lines between following start points include the 2D space
     * @param vectorsTwo Second pairs of vector start points and end points that create a 2D space, ordered such that left halves of lines between following start points include the 2D space
     * @return True if 2D intersection is non-empty, otherwise false
     */
    // @Deprecated
    // public static boolean checkIntersection2D(List<Map.Entry<Vec3, Vec3>> vectorsOne, List<Map.Entry<Vec3, Vec3>> vectorsTwo) {
    //     //TODO: Function is unnecessary with three dimensional collision detection
    //     // If any list is empty, return false
    //     if (vectorsOne.isEmpty() || vectorsTwo.isEmpty()) {
    //         return false;
    //     }

    //     // Convert vector start and end pairs to lists
    //     List<Vector2D> listOne = new LinkedList<>();
    //     List<Vector2D> listTwo = new LinkedList<>();

    //     for (Map.Entry<Vec3, Vec3> entry : vectorsOne) {
    //         // Vectors must have at least dimension 2, otherwise result is always false
    //         if (entry.getKey().getDimension() < 2 || entry.getValue().getDimension() < 2) {
    //             return false;
    //         }

    //         Vector2D vertexOne = new Vector2D(entry.getKey().getEntry(0), entry.getKey().getEntry(1));
    //         listOne.add(vertexOne);
    //     }

    //     for (Map.Entry<Vec3, Vec3> entry : vectorsTwo) {
    //         // Vectors must have at least dimension 2, otherwise result is always false
    //         if (entry.getKey().getDimension() < 2 || entry.getValue().getDimension() < 2) {
    //             return false;
    //         }

    //         Vector2D vertexOne = new Vector2D(entry.getKey().getEntry(0), entry.getKey().getEntry(1));
    //         listTwo.add(vertexOne);
    //     }

    //     // From lists create regions
    //     Region<Euclidean2D> regionOne = new PolygonsSet(1.0e-5, listOne.toArray(new Vector2D[listOne.size()]));
    //     Region<Euclidean2D> regionTwo = new PolygonsSet(1.0e-5, listTwo.toArray(new Vector2D[listTwo.size()]));

    //     // Check intersection for emptiness
    //     RegionFactory<Euclidean2D> regionFactory = new RegionFactory<>();
    //     Region<Euclidean2D> regionIntersection = regionFactory.intersection(regionOne, regionTwo);
    //     return !regionIntersection.isEmpty();
    // }

    

}
