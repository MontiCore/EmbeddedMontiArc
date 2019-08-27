/* (c) https://github.com/MontiCore/monticore */
package simulation.util;

import org.apache.commons.math3.geometry.euclidean.twod.Euclidean2D;
import org.apache.commons.math3.geometry.euclidean.twod.PolygonsSet;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.apache.commons.math3.geometry.partitioning.Region;
import org.apache.commons.math3.geometry.partitioning.RegionFactory;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.random.RandomDataGenerator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Class that provides static access to common math operations
 */
public final class MathHelper {

    /**
     * Empty non accessible constructor, there is no instance of this class
     */
    private MathHelper() {}

    /**
     * Function that checks for vector equality allowing a small fixed error factor because of numeric issues
     *
     * @param vector1 First vector to be checked for fuzzy equality
     * @param vector2 Second vector to be checked for fuzzy equality
     * @param threshold Threshold value for check of norm, e.g. a value such as 0.00000001
     * @return Boolean indicating fuzzy equality of input vectors
     */
    public static boolean vectorEquals(RealVector vector1, RealVector vector2, double threshold) {
        // Simple check to avoid computations if possible
        if (vector1.getDimension() != vector2.getDimension() || vector1.getDimension() == 0) {
            return false;
        }

        // If vectors are nearly equal, norm for all entries of difference vectors must be near 0
        RealVector diffVector = vector1.subtract(vector2);
        double norm = diffVector.getNorm();
        return (norm < threshold);
    }

    /**
     * Function that checks for matrix equality allowing a small fixed error factor because of numeric issues
     *
     * @param matrix1 First matrix to be checked for fuzzy equality
     * @param matrix2 Second matrix to be checked for fuzzy equality
     * @param threshold Threshold value for check of norm, e.g. a value such as 0.0000001
     * @return Boolean indicating fuzzy equality of input matrices
     */
    public static boolean matrixEquals(RealMatrix matrix1, RealMatrix matrix2, double threshold) {
        // Simple check to avoid computations if possible
        if (matrix1.getColumnDimension() != matrix2.getColumnDimension() ||
                matrix1.getRowDimension() != matrix2.getColumnDimension() ||
                matrix1.getColumnDimension() == 0 || matrix1.getRowDimension() == 0) {
            return false;
        }

        // If matrices are nearly equal, norm for all entries of difference matrix must be near 0
        RealMatrix diffMatrix = matrix1.subtract(matrix2);
        double norm = diffMatrix.getFrobeniusNorm();
        return (norm < threshold);
    }

    /**
     * Function that computes the inverse of an input matrix, if possible
     * Perform some simple checks before actually computing the matrix inverse
     *
     * @param matrix Matrix that is used for the inverse computation, should be square and invertible matrix
     * @return Inverse matrix of the input matrix
     */
    public static RealMatrix matrixInvert(RealMatrix matrix) {
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

    /**
     * Function that performs a re-orthonormalization for a given rotation matrix
     *
     * @param matrix Input 3x3 rotation matrix that is used in re-orthonormalization
     * @return Result 3x3 rotation matrix that is now orthonormal again
     */
    public static RealMatrix matrixReOrthonormalize(RealMatrix matrix) {
        // Checks for valid input
        if (matrix.getColumnDimension() != 3 || !matrix.isSquare()) {
            throw new IllegalArgumentException("Matrix " + matrix + " should be a 3x3 matrix.");
        }
        if (matrix.getColumnVector(0).getNorm() == 0 || matrix.getColumnVector(1).getNorm() == 0 || matrix.getColumnVector(2).getNorm() == 0){
            throw new IllegalArgumentException("Matrix " + matrix + " should have no zero-norm columns.");
        }

        // Prepare variables

        // Approach: Gram Schmidt re-orthonormalization
        RealVector v0 = matrix.getColumnVector(0);
        RealVector v1 = matrix.getColumnVector(1);
        RealVector v2 = matrix.getColumnVector(2);

        v0 = v0.mapDivide(v0.getNorm());

        v1 = v1.subtract(v0.mapMultiply(v0.dotProduct(v1)));
        v1 = v1.mapDivide(v1.getNorm());

        v2 = v2.subtract(v0.mapMultiply(v0.dotProduct(v2))).subtract(v1.mapMultiply(v1.dotProduct(v2)));
        v2 = v2.mapDivide(v2.getNorm());

        matrix.setColumnVector(0, v0);
        matrix.setColumnVector(1, v1);
        matrix.setColumnVector(2, v2);
        return matrix;
    }

    /**
     * Function that computes the cross product of two 3-dimensional vectors
     * Only supports 3-dimensional vectors and 3x3 matrices as result
     *
     * @param vector1 First 3-dimensional vector that is used in cross product computation
     * @param vector2 Second 3-dimensional vector that is used in cross product computation
     * @return 3-dimensional cross product for the input 3-dimensional vectors
     */
    public static RealVector crossProduct(RealVector vector1, RealVector vector2) {
        // Check for valid input
        if(vector1.getDimension() != 3){
            throw new IllegalArgumentException("First vector " + vector1 + " should be a three dimensional vector.");
        }
        if(vector2.getDimension() != 3){
            throw new IllegalArgumentException("Second vector " + vector2 + " should be a three dimensional vector.");
        }

        // This is also an example of how to use the vector3DToCrossProductMatrix function
        RealMatrix crossProductMatrixVector1 = vectorToCrossProductMatrix(vector1);
        return crossProductMatrixVector1.operate(vector2);
    }

    /**
     * Function that converts a vector to a matrix such that the matrix multiplied with another vector is the cross product
     * Perform some simple checks before actually computing the matrix
     * Only supports 3-dimensional vectors and 3x3 matrices as result
     *
     * @param vector 3-dimensional vector for which the corresponding cross product matrix should be computed
     * @return 3x3 cross product matrix for the input 3-dimensional vector
     */
    public static RealMatrix vectorToCrossProductMatrix(RealVector vector) {
        // Check for valid input
        if(vector.getDimension() != 3){
            throw new IllegalArgumentException("Vector " + vector + " should be a three dimensional vector.");
        }

        double[][] matrixEntries = {{0.0, -vector.getEntry(2), vector.getEntry(1)}, {vector.getEntry(2), 0.0, -vector.getEntry(0)}, {-vector.getEntry(1), vector.getEntry(0), 0.0}};
        return new BlockRealMatrix(matrixEntries);
    }

    /**
     * Function that computes the angle between two three dimensional vectors
     *
     * @param vector1 First 3-dimensional vector that is used in the angle computation
     * @param vector2 Second 3-dimensional vector that is used in the angle computation
     * @return Angle between the two 3-dimensional vectors
     */
    public static double angle(RealVector vector1, RealVector vector2){
        // Checks for valid input
        if(vector1.getDimension() != 3){
            throw new IllegalArgumentException("First vector " + vector1 + " should be a three dimensional vector.");
        }
        if(vector2.getDimension() != 3){
            throw new IllegalArgumentException("Second vector " + vector2 + " should be a three dimensional vector.");
        }
        double norm1 = vector1.getNorm();
        if(norm1 == 0){
            throw new IllegalArgumentException("Second vector " + vector2 + " should not have a zero norm.");
        }
        double norm2 = vector2.getNorm();
        if(norm2 == 0){
            throw new IllegalArgumentException("Second vector " + vector2 + " should not have a zero norm.");
        }
        double dotProduct = vector1.dotProduct(vector2);
        return Math.acos(dotProduct / (norm1 * norm2));
    }

    /**
     * Function that converts a 3-dimensional vector into a Vector3D
     *
     * @param vector 3-dimensional vector that is converted into a Vector3D
     * @return Vector3D with the same values as the given vector
     */
    public static Vector3D realTo3D(RealVector vector){
        // Checks for valid input
        if(vector.getDimension() != 3){
            throw new IllegalArgumentException("Vector " + vector + " should be a three dimensional vector.");
        }
        return  new Vector3D(vector.getEntry(0), vector.getEntry(1), vector.getEntry(2));
    }

    /**
     * Function that checks if two oriented bounding boxes intersect
     *
     * @param boxOne First bounding box
     * @param boxTwo Second bounding box
     * @return True if the boxes intersect, otherwise false
     */
    public static boolean checkIntersection(OrientedBoundingBox boxOne, OrientedBoundingBox boxTwo) {
        // Using separating axis theorem the bounding boxes intersect iff no separating plane exists for these 15 cases
        return !(existsSeparatingPlane(boxOne.getLocalAxisX(), boxOne, boxTwo) ||
                existsSeparatingPlane(boxOne.getLocalAxisY(), boxOne, boxTwo) ||
                existsSeparatingPlane(boxOne.getLocalAxisZ(), boxOne, boxTwo) ||
                existsSeparatingPlane(boxTwo.getLocalAxisX(), boxOne, boxTwo) ||
                existsSeparatingPlane(boxTwo.getLocalAxisY(), boxOne, boxTwo) ||
                existsSeparatingPlane(boxTwo.getLocalAxisZ(), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisX(), boxTwo.getLocalAxisX()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisX(), boxTwo.getLocalAxisY()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisX(), boxTwo.getLocalAxisZ()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisY(), boxTwo.getLocalAxisX()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisY(), boxTwo.getLocalAxisY()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisY(), boxTwo.getLocalAxisZ()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisZ(), boxTwo.getLocalAxisX()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisZ(), boxTwo.getLocalAxisY()), boxOne, boxTwo) ||
                existsSeparatingPlane(crossProduct(boxOne.getLocalAxisZ(), boxTwo.getLocalAxisZ()), boxOne, boxTwo));
    }

    /**
     * Function that checks whether there exists a separating plane between 2 oriented bounding boxes
     *
     * @param separatingAxis Vector to which the separating plane should be orthogonal to
     * @param boxOne First oriented bounding box
     * @param boxTwo Second oriented bounding box
     * @return True if separating plane exists, otherwise false
     */
    private static boolean existsSeparatingPlane(RealVector separatingAxis, OrientedBoundingBox boxOne, OrientedBoundingBox boxTwo) {
        RealVector posDifference = boxTwo.getPosition().subtract(boxOne.getPosition());
        return Math.abs(posDifference.dotProduct(separatingAxis)) >
                (Math.abs(boxOne.getLocalAxisX().mapMultiply(boxOne.getHalfWidth()).dotProduct(separatingAxis)) +
                        Math.abs(boxOne.getLocalAxisY().mapMultiply(boxOne.getHalfLength()).dotProduct(separatingAxis)) +
                        Math.abs(boxOne.getLocalAxisZ().mapMultiply(boxOne.getHalfHeight()).dotProduct(separatingAxis)) +
                        Math.abs(boxTwo.getLocalAxisX().mapMultiply(boxTwo.getHalfWidth()).dotProduct(separatingAxis)) +
                        Math.abs(boxTwo.getLocalAxisY().mapMultiply(boxTwo.getHalfLength()).dotProduct(separatingAxis)) +
                        Math.abs(boxTwo.getLocalAxisZ().mapMultiply(boxTwo.getHalfHeight()).dotProduct(separatingAxis)));
    }

    /**
     * Function that checks if two 2D spaces defined by real vectors have a non empty 2D space intersection
     *
     * @param vectorsOne First pairs of vector start points and end points that create a 2D space, ordered such that left halves of lines between following start points include the 2D space
     * @param vectorsTwo Second pairs of vector start points and end points that create a 2D space, ordered such that left halves of lines between following start points include the 2D space
     * @return True if 2D intersection is non-empty, otherwise false
     */
    @Deprecated
    public static boolean checkIntersection2D(List<Map.Entry<RealVector, RealVector>> vectorsOne, List<Map.Entry<RealVector, RealVector>> vectorsTwo) {
        //TODO: Function is unnecessary with three dimensional collision detection
        // If any list is empty, return false
        if (vectorsOne.isEmpty() || vectorsTwo.isEmpty()) {
            return false;
        }

        // Convert vector start and end pairs to lists
        List<Vector2D> listOne = new LinkedList<>();
        List<Vector2D> listTwo = new LinkedList<>();

        for (Map.Entry<RealVector, RealVector> entry : vectorsOne) {
            // Vectors must have at least dimension 2, otherwise result is always false
            if (entry.getKey().getDimension() < 2 || entry.getValue().getDimension() < 2) {
                return false;
            }

            Vector2D vertexOne = new Vector2D(entry.getKey().getEntry(0), entry.getKey().getEntry(1));
            listOne.add(vertexOne);
        }

        for (Map.Entry<RealVector, RealVector> entry : vectorsTwo) {
            // Vectors must have at least dimension 2, otherwise result is always false
            if (entry.getKey().getDimension() < 2 || entry.getValue().getDimension() < 2) {
                return false;
            }

            Vector2D vertexOne = new Vector2D(entry.getKey().getEntry(0), entry.getKey().getEntry(1));
            listTwo.add(vertexOne);
        }

        // From lists create regions
        Region<Euclidean2D> regionOne = new PolygonsSet(1.0e-5, listOne.toArray(new Vector2D[listOne.size()]));
        Region<Euclidean2D> regionTwo = new PolygonsSet(1.0e-5, listTwo.toArray(new Vector2D[listTwo.size()]));

        // Check intersection for emptiness
        RegionFactory<Euclidean2D> regionFactory = new RegionFactory<>();
        Region<Euclidean2D> regionIntersection = regionFactory.intersection(regionOne, regionTwo);
        return !regionIntersection.isEmpty();
    }

    /**
     * Function that generates uniformly distributed random long values in a specified interval
     *
     * @param lower Lower end of the interval, included
     * @param upper Upper end of the interval, included
     * @return Uniformly random long value within the interval
     */
    public static long randomLong(long lower, long upper) {
        // Check for valid input
        if(lower > upper){
            throw new IllegalArgumentException("Lower end " + lower + " should not be higher as upper end " + upper + ".");
        }
        // For same value return the same value
        if (lower == upper) {
            return lower;
        }

        // Otherwise generate random value
        RandomDataGenerator random = new RandomDataGenerator();
        return random.nextLong(lower, upper);
    }

    /**
     * Function that generates uniformly distributed random int values in a specified interval
     *
     * @param lower Lower end of the interval, included
     * @param upper Upper end of the interval, included
     * @return Uniformly random int value within the interval
     */
    public static int randomInt(int lower, int upper) {
        // Check for valid input
        if(lower > upper){
            throw new IllegalArgumentException("Lower end " + lower + " should not be higher as upper end " + upper + ".");
        }
        // For same value return the same value
        if (lower == upper) {
            return lower;
        }

        // Otherwise generate random value
        RandomDataGenerator random = new RandomDataGenerator();
        return random.nextInt(lower, upper);
    }
}
