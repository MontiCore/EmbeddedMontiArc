/**
 *
 * /* (c) https://github.com/MontiCore/monticore */
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package simulation.util;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.RotationConvention;
import org.apache.commons.math3.geometry.euclidean.threed.RotationOrder;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.linear.*;
import org.junit.*;
import java.util.AbstractMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import static org.junit.Assert.*;

/**
 * Class that tests the MathHelper class
 */
public class MathHelperTest {

    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    @Test
    public void vectorEqualsTest() {
        // Test equal case
        RealVector vector1 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        RealVector vector2 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        assertTrue(MathHelper.vectorEquals(vector1, vector2, 0.0000001));

        // Test almost equal case
        vector1 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        vector2 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        vector2 = vector2.mapAdd(0.00000002);
        assertTrue(MathHelper.vectorEquals(vector1, vector2, 0.0000001));

        // Test not equal case
        vector1 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        vector2 = new ArrayRealVector(new double[]{0.2537, 87.258, 0.05739});
        assertFalse(MathHelper.vectorEquals(vector1, vector2, 0.0000001));
    }

    @Test
    public void matrixEqualsTest() {
        // Test equal case
        RealMatrix matrix1 = new BlockRealMatrix(new double[][]{{23.4, 3.2, 9.3}, {7.5, 9.8, 29.3}, {1.2, 0.8346, 238.3}});
        RealMatrix matrix2 = new BlockRealMatrix(new double[][]{{23.4, 3.2, 9.3}, {7.5, 9.8, 29.3}, {1.2, 0.8346, 238.3}});
        assertTrue(MathHelper.matrixEquals(matrix1, matrix2, 0.0000001));

        // Test almost equal case
        matrix1 = new BlockRealMatrix(new double[][]{{23.4, 3.2, 9.3}, {7.5, 9.8, 29.3}, {1.2, 0.8346, 238.3}});
        matrix2 = new BlockRealMatrix(new double[][]{{23.4, 3.2, 9.3}, {7.5, 9.8, 29.3}, {1.2, 0.8346, 238.3}});
        matrix2 = matrix2.scalarAdd(0.00000002);
        assertTrue(MathHelper.matrixEquals(matrix1, matrix2, 0.0000001));

        // Test not equal case
        matrix1 = new BlockRealMatrix(new double[][]{{23.4, 3.2, 9.3}, {7.5, 9.8, 29.3}, {1.2, 0.8346, 238.3}});
        matrix2 = new BlockRealMatrix(new double[][]{{0.034, 0.3, 0.853}, {0.82356, 0.378, 0.385}, {1.3534, 0.1235, 23.248}});
        assertFalse(MathHelper.matrixEquals(matrix1, matrix2, 0.0000001));
    }

    @Test
    public void matrixInvertNormal() {
        // Invert - Invert should be identity
        RealMatrix matrix = new BlockRealMatrix(new double[][]{{23.4, 3.2, 9.3}, {7.5, 9.8, 29.3}, {1.2, 0.8346, 238.3}});
        RealMatrix inverse1 = MathHelper.matrixInvert(matrix);
        RealMatrix inverse2 = MathHelper.matrixInvert(inverse1);
        assertTrue(MathHelper.matrixEquals(matrix, inverse2, 0.0000001));

        // Invert - Invert should be identity
        matrix = new BlockRealMatrix(new double[][]{{0.034, 0.3, 0.853}, {0.82356, 0.378, 0.385}, {1.3534, 0.1235, 23.248}});
        inverse1 = MathHelper.matrixInvert(matrix);
        inverse2 = MathHelper.matrixInvert(inverse1);
        assertTrue(MathHelper.matrixEquals(matrix, inverse2, 0.0000001));
    }

    @Test(expected = IllegalArgumentException.class)
    public void matrixInvertNotSquare() {
        RealMatrix matrix = new BlockRealMatrix(new double[][]{{0.034, 0.3}, {0.82356, 0.378}, {1.3534, 0.1235}});
        MathHelper.matrixInvert(matrix);
    }

    @Test(expected = IllegalArgumentException.class)
    public void matrixInvertSingular() {
        RealMatrix matrix = new BlockRealMatrix(new double[][]{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
        MathHelper.matrixInvert(matrix);
    }

    //TODO: matrix re-orthonormalize

    @Test
    public void crossProductNormal() {
        RealVector vector1 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        RealVector vector2 = new ArrayRealVector(new double[]{0.2537, 87.258, 0.05739});
        RealVector crossProduct = MathHelper.crossProduct(vector1, vector2);
        RealVector referenceCrossProduct = new ArrayRealVector(3);
        referenceCrossProduct.setEntry(0, vector1.getEntry(1)*vector2.getEntry(2) - vector1.getEntry(2)*vector2.getEntry(1));
        referenceCrossProduct.setEntry(1, vector1.getEntry(2)*vector2.getEntry(0) - vector1.getEntry(0)*vector2.getEntry(2));
        referenceCrossProduct.setEntry(2, vector1.getEntry(0)*vector2.getEntry(1) - vector1.getEntry(1)*vector2.getEntry(0));
        assertTrue(MathHelper.vectorEquals(referenceCrossProduct, crossProduct, 0.00000001));
    }

    @Test(expected = IllegalArgumentException.class)
    public void crossProductFirstWrongDimension(){
        RealVector vector1 = new ArrayRealVector(new double[]{23.237, 239.258});
        RealVector vector2 = new ArrayRealVector(new double[]{0.2537, 87.258, 0.05739});
        MathHelper.crossProduct(vector1, vector2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void crossProductSecondWrongDimension(){
        RealVector vector1 = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        RealVector vector2 = new ArrayRealVector(new double[]{0.2537, 87.258});
        MathHelper.crossProduct(vector1, vector2);
    }

    @Test
    public void vectorToCrossProductMatrix() {
        RealVector vector = new ArrayRealVector(new double[]{23.237, 239.258, 0.2374});
        RealMatrix crossProductMatrix = MathHelper.vectorToCrossProductMatrix(vector);
        RealMatrix referenceCrossProductMatrix = new BlockRealMatrix(3, 3);
        referenceCrossProductMatrix.setEntry(0, 0, 0.0);
        referenceCrossProductMatrix.setEntry(0, 1, -vector.getEntry(2));
        referenceCrossProductMatrix.setEntry(0, 2, vector.getEntry(1));
        referenceCrossProductMatrix.setEntry(1, 0, vector.getEntry(2));
        referenceCrossProductMatrix.setEntry(1, 1, 0.0);
        referenceCrossProductMatrix.setEntry(1, 2, -vector.getEntry(0));
        referenceCrossProductMatrix.setEntry(2, 0, -vector.getEntry(1));
        referenceCrossProductMatrix.setEntry(2, 1, vector.getEntry(0));
        referenceCrossProductMatrix.setEntry(2, 2, 0.0);
        assertTrue(MathHelper.matrixEquals(referenceCrossProductMatrix, crossProductMatrix, 0.00000001));
    }

    @Test(expected = IllegalArgumentException.class)
    public void vectorToCrossProductMatrixWrongDimension(){
        RealVector vector = new ArrayRealVector(new double[]{0.2537, 87.258, 0.05739, 23.5295});
        MathHelper.vectorToCrossProductMatrix(vector);
    }

    @Test
    public void angleNormal(){
        // Test normal case
        RealVector vector1 = new ArrayRealVector(new double[]{1.245, -12.73, 19.0});
        RealVector vector2 = new ArrayRealVector(new double[]{-3.79, 26.237, 0.67});
        double angle = MathHelper.angle(vector1, vector2);
        double referenceAngle = Math.acos((vector1.dotProduct(vector2))/(vector1.getNorm() * vector2.getNorm()));
        assertEquals(referenceAngle, angle, 0);

        // Test equal vectors
        vector1 = new ArrayRealVector(new double[]{1.245, -12.73, 19.0});
        vector2 = new ArrayRealVector(new double[]{1.245, -12.73, 19.0});
        angle = MathHelper.angle(vector1, vector2);
        referenceAngle = Math.acos((vector1.dotProduct(vector2))/(vector1.getNorm() * vector2.getNorm()));
        assertEquals(referenceAngle, angle, 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void angleFirstWrongDimension(){
        RealVector vector1 = new ArrayRealVector(new double[]{1.245, -12.73});
        RealVector vector2 = new ArrayRealVector(new double[]{-3.79, 26.237, 0.67});
        double angle = MathHelper.angle(vector1, vector2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void angleSecondWrongDimension(){
        RealVector vector1 = new ArrayRealVector(new double[]{1.245, -12.73, 19.0});
        RealVector vector2 = new ArrayRealVector(new double[]{-3.79, 26.237});
        double angle = MathHelper.angle(vector1, vector2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void angleFirstZeroNorm(){
        RealVector vector1 = new ArrayRealVector(3);
        RealVector vector2 = new ArrayRealVector(new double[]{-3.79, 26.237, 0.67});
        double angle = MathHelper.angle(vector1, vector2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void angleSecondZeroNorm(){
        RealVector vector1 = new ArrayRealVector(new double[]{1.245, -12.73, 19.0});
        RealVector vector2 = new ArrayRealVector(3);
        double angle = MathHelper.angle(vector1, vector2);
    }

    @Test
    public void realTo3DNormal(){
        RealVector vector = new ArrayRealVector(new double[]{19.02, -29.1, -0.02});
        Vector3D vector3D = MathHelper.realTo3D(vector);
        Vector3D referenceVector3D = new Vector3D(19.02, -29.1, -0.02);
        assertEquals(referenceVector3D.getX(), vector3D.getX(), 0);
        assertEquals(referenceVector3D.getY(), vector3D.getY(), 0);
        assertEquals(referenceVector3D.getZ(), vector3D.getZ(), 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void realTo3DWrongDimension(){
        RealVector vector = new ArrayRealVector(new double[]{19.02, -29.1});
        Vector3D vector3D = MathHelper.realTo3D(vector);
    }
    
    @Test
    public void checkIntersectionTest() {
        RealMatrix noRotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, 0.0).getMatrix());

        OrientedBoundingBox box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 4, 5, noRotation);
        OrientedBoundingBox box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {1, 0, 1}), 3, 4, 5, noRotation);
        assertTrue(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {-10, 0, 0}), 3, 4, 5, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {1, 0, 1}), 3, 4, 5, noRotation);
        assertFalse(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 4, 5, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {10, 0, 1}), 3, 4, 5, noRotation);
        assertFalse(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 4, 5, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {3, 0, 0}), 3, 4, 5, noRotation);
        assertTrue(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 3, 3, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {3.1, 0, 0}), 3, 3, 3, noRotation);
        assertFalse(MathHelper.checkIntersection(box1, box2));

        RealMatrix rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, Math.PI / 4.0).getMatrix());
        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 3, 3, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {3.1, 0, 0}), 3, 3, 3, rotation);
        assertTrue(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 3, 3, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 3.1, 0}), 3, 3, 3, noRotation);
        assertFalse(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 3.1}), 3, 3, 3, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 3, 3, noRotation);
        assertFalse(MathHelper.checkIntersection(box1, box2));

        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 3}), 3, 3, 3, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 3, 3, noRotation);
        assertTrue(MathHelper.checkIntersection(box1, box2));

        rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, Math.PI / 2.0).getMatrix());
        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 3, 6, 2, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {4.5, 0, 0}), 3, 6, 2, rotation);
        assertTrue(MathHelper.checkIntersection(box1, box2));

        rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, 0.0, 0.0, Math.PI).getMatrix());
        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 2, 4, 6, noRotation);
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 5, 0}), 2, 4, 6, rotation);
        assertFalse(MathHelper.checkIntersection(box1, box2));

        rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, Math.PI / 4.0, 0.0, 0.0).getMatrix());
        box1 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 0, 0}), 2, 4, 6, rotation);
        rotation = new BlockRealMatrix(new Rotation(RotationOrder.XYZ, RotationConvention.VECTOR_OPERATOR, -Math.PI / 4.0, 0.0, Math.PI).getMatrix());
        box2 = new OrientedBoundingBox(new ArrayRealVector(new double[] {0, 5, 0}), 2, 4, 6, rotation);
        assertTrue(MathHelper.checkIntersection(box1, box2));
    }

    @Test
    public void checkIntersection2DTest() {
        //TODO: Function is unnecessary with three dimensional collision detection
        List<Map.Entry<RealVector, RealVector>> list1 = new LinkedList<>();
        Map.Entry<RealVector, RealVector> e1 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{0.0, 0.0, 0.0}), new ArrayRealVector(new double[]{4.0, 0.0, 0.0}));
        Map.Entry<RealVector, RealVector> e2 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{4.0, 0.0, 0.0}), new ArrayRealVector(new double[]{4.0, 4.0, 0.0}));
        Map.Entry<RealVector, RealVector> e3 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{4.0, 4.0, 0.0}), new ArrayRealVector(new double[]{0.0, 4.0, 0.0}));
        Map.Entry<RealVector, RealVector> e4 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{0.0, 4.0, 0.0}), new ArrayRealVector(new double[]{0.0, 0.0, 0.0}));
        list1.add(e1); list1.add(e2); list1.add(e3); list1.add(e4);

        List<Map.Entry<RealVector, RealVector>> list2 = new LinkedList<>();
        assertFalse(MathHelper.checkIntersection2D(list1, list2));

        Map.Entry<RealVector, RealVector> e5 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{0.0, 10.0, 0.0}), new ArrayRealVector(new double[]{4.0, 10.0, 0.0}));
        Map.Entry<RealVector, RealVector> e6 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{4.0, 10.0, 0.0}), new ArrayRealVector(new double[]{4.0, 14.0, 0.0}));
        Map.Entry<RealVector, RealVector> e7 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{4.0, 14.0, 0.0}), new ArrayRealVector(new double[]{0.0, 14.0, 0.0}));
        Map.Entry<RealVector, RealVector> e8 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{0.0, 14.0, 0.0}), new ArrayRealVector(new double[]{0.0, 10.0, 0.0}));
        list2.add(e5); list2.add(e6); list2.add(e7); list2.add(e8);
        assertFalse(MathHelper.checkIntersection2D(list1, list2));

        List<Map.Entry<RealVector, RealVector>> list3 = new LinkedList<>();
        Map.Entry<RealVector, RealVector> e9 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{2.0, 12.0, 0.0}), new ArrayRealVector(new double[]{-3.0, 7.0, 0.0}));
        Map.Entry<RealVector, RealVector> e10 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{-3.0, 7.0, 0.0}), new ArrayRealVector(new double[]{2.0, 2.0, 0.0}));
        Map.Entry<RealVector, RealVector> e11 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{2.0, 2.0, 0.0}), new ArrayRealVector(new double[]{7.0, 7.0, 0.0}));
        Map.Entry<RealVector, RealVector> e12 = new AbstractMap.SimpleEntry<RealVector, RealVector>(new ArrayRealVector(new double[]{7.0, 7.0, 0.0}), new ArrayRealVector(new double[]{2.0, 12.0, 0.0}));
        list3.add(e9); list3.add(e10); list3.add(e11); list3.add(e12);
        assertTrue(MathHelper.checkIntersection2D(list1, list3));
        assertTrue(MathHelper.checkIntersection2D(list2, list3));
    }

    @Test
    public void randomLongNormal() {
        // Test small range case
        long result = MathHelper.randomLong(-10L, 10L);
        assertTrue(result >= -10L && result <= 10L);

        // Test large range case
        result = MathHelper.randomLong(Long.MIN_VALUE / 2L, Long.MAX_VALUE / 2L);
        assertTrue(result >= Long.MIN_VALUE / 2L && result <= Long.MAX_VALUE / 2L);

        // Test zero range test at different values
        assertEquals(0L, MathHelper.randomLong(0L, 0L));
        assertEquals(-1L, MathHelper.randomLong(-1L, -1L));
        assertEquals(Long.MIN_VALUE, MathHelper.randomLong(Long.MIN_VALUE, Long.MIN_VALUE));
        assertEquals(Long.MAX_VALUE, MathHelper.randomLong(Long.MAX_VALUE, Long.MAX_VALUE));
        assertEquals(20L, MathHelper.randomLong(20L, 20L));
    }

    @Test(expected = IllegalArgumentException.class)
    public void randomLongFailMinMax(){
        MathHelper.randomLong(20L, 10L);
    }

    @Test
    public void randomIntTest() {
        // Test small range case
        long result = MathHelper.randomInt(-10, 10);
        assertTrue(result >= -10 && result <= 10);

        // Test large range case
        result = MathHelper.randomInt(Integer.MIN_VALUE / 2, Integer.MAX_VALUE / 2);
        assertTrue(result >= Integer.MIN_VALUE / 2 && result <= Integer.MAX_VALUE / 2);

        // Test zero range test at different values
        assertEquals(0, MathHelper.randomInt(0, 0));
        assertEquals(-1, MathHelper.randomInt(-1, -1));
        assertEquals(Integer.MIN_VALUE, MathHelper.randomInt(Integer.MIN_VALUE, Integer.MIN_VALUE));
        assertEquals(Integer.MAX_VALUE, MathHelper.randomInt(Integer.MAX_VALUE, Integer.MAX_VALUE));
        assertEquals(20, MathHelper.randomInt(20, 20));
    }

    @Test(expected = IllegalArgumentException.class)
    public void randomIngFailMinMax(){
        MathHelper.randomInt(20, 10);
    }
}
