package de.rwth.montisim.commons.utils;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.utils.PolygonConvexer.ConversionError;

public class PolygonConvexerTest {
    
    @Test
    public void noConversionCCWTest() throws ConversionError {
        Polygon p = new Polygon();
        p.points.add(new Vec2(-2.2,1.9));
        p.points.add(new Vec2(-1.1,-2.1));
        p.points.add(new Vec2(3.05,-2.4));
        p.points.add(new Vec2(4.9,-0.2));
        p.points.add(new Vec2(3.5,2.7));
        p.points.add(new Vec2(1.2,3.2));

        List<Polygon> parts = new PolygonConvexer(p).getParts();

        List<Polygon> expectedParts = new ArrayList<>();
        expectedParts.add(p);

        assertParts(parts, expectedParts);

        //p.points.add(new Vec2(,));
    }

    @Test
    public void noConversionCWTest() throws ConversionError {
        Polygon p = new Polygon();
        p.points.add(new Vec2(1.2,3.2));
        p.points.add(new Vec2(3.5,2.7));
        p.points.add(new Vec2(4.9,-0.2));
        p.points.add(new Vec2(3.05,-2.4));
        p.points.add(new Vec2(-1.1,-2.1));
        p.points.add(new Vec2(-2.2,1.9));

        List<Polygon> parts = new PolygonConvexer(p).getParts();

        List<Polygon> expectedParts = new ArrayList<>();
        Polygon expectedPolygon = new Polygon();
        expectedPolygon.points.add(new Vec2(-2.2,1.9));
        expectedPolygon.points.add(new Vec2(-1.1,-2.1));
        expectedPolygon.points.add(new Vec2(3.05,-2.4));
        expectedPolygon.points.add(new Vec2(4.9,-0.2));
        expectedPolygon.points.add(new Vec2(3.5,2.7));
        expectedPolygon.points.add(new Vec2(1.2,3.2));
        expectedParts.add(expectedPolygon);

        assertParts(parts, expectedParts);
    }

    @Test
    public void concave1CCWtest() throws ConversionError {
        Polygon p = new Polygon();
        p.points.add(new Vec2(-4.2,0.1));
        p.points.add(new Vec2(-2.7,-3.4));
        p.points.add(new Vec2(-0.8,-1.5));
        p.points.add(new Vec2(1.6,-1.7));
        p.points.add(new Vec2(3.3,-3.9));
        p.points.add(new Vec2(4.8,-0.5));
        p.points.add(new Vec2(2.2,2.5));
        p.points.add(new Vec2(-2,2.7));

        
        List<Polygon> parts = new PolygonConvexer(p).getParts();

        
        List<Polygon> expectedParts = new ArrayList<>();
        Polygon ep1 = new Polygon();
        ep1.points.add(new Vec2(2.2,2.5));
        ep1.points.add(new Vec2(-2,2.7));
        ep1.points.add(new Vec2(-4.2,0.1));
        ep1.points.add(new Vec2(-2.7,-3.4));
        ep1.points.add(new Vec2(-0.8,-1.5));
        expectedParts.add(ep1);
        Polygon ep2 = new Polygon();
        ep2.points.add(new Vec2(4.8,-0.5));
        ep2.points.add(new Vec2(2.2,2.5));
        ep2.points.add(new Vec2(-0.8,-1.5));
        ep2.points.add(new Vec2(1.6,-1.7));
        expectedParts.add(ep2);
        Polygon ep3 = new Polygon();
        ep3.points.add(new Vec2(1.6,-1.7));
        ep3.points.add(new Vec2(3.3,-3.9));
        ep3.points.add(new Vec2(4.8,-0.5));
        expectedParts.add(ep3);

        
        assertParts(parts, expectedParts);
    }

    @Test
    public void concave1CWtest() throws ConversionError {
        Polygon p = new Polygon();

        p.points.add(new Vec2(-2,2.7));
        p.points.add(new Vec2(2.2,2.5));
        p.points.add(new Vec2(4.8,-0.5));
        p.points.add(new Vec2(3.3,-3.9));
        p.points.add(new Vec2(1.6,-1.7));
        p.points.add(new Vec2(-0.8,-1.5));
        p.points.add(new Vec2(-2.7,-3.4));
        p.points.add(new Vec2(-4.2,0.1));

        
        List<Polygon> parts = new PolygonConvexer(p).getParts();

        
        List<Polygon> expectedParts = new ArrayList<>();
        Polygon ep1 = new Polygon();
        ep1.points.add(new Vec2(2.2,2.5));
        ep1.points.add(new Vec2(-2,2.7));
        ep1.points.add(new Vec2(-4.2,0.1));
        ep1.points.add(new Vec2(-2.7,-3.4));
        ep1.points.add(new Vec2(-0.8,-1.5));
        expectedParts.add(ep1);
        Polygon ep2 = new Polygon();
        ep2.points.add(new Vec2(4.8,-0.5));
        ep2.points.add(new Vec2(2.2,2.5));
        ep2.points.add(new Vec2(-0.8,-1.5));
        ep2.points.add(new Vec2(1.6,-1.7));
        expectedParts.add(ep2);
        Polygon ep3 = new Polygon();
        ep3.points.add(new Vec2(1.6,-1.7));
        ep3.points.add(new Vec2(3.3,-3.9));
        ep3.points.add(new Vec2(4.8,-0.5));
        expectedParts.add(ep3);

        
        assertParts(parts, expectedParts);
    }

    @Test
    public void concave2CWtest() throws ConversionError {
        Polygon p = new Polygon();

        p.points.add(new Vec2(-1,1.8)); // 0
        p.points.add(new Vec2(-1.1,-0.8)); // 1
        p.points.add(new Vec2(-0.4, 0.5)); // 2
        p.points.add(new Vec2(-0.8, -1.1)); // 3
        p.points.add(new Vec2(1.4, -0.9)); // 4
        p.points.add(new Vec2(0.2,-0.8)); // 5
        p.points.add(new Vec2(-0.1,0.1)); // 6
        p.points.add(new Vec2(1.9,1.0)); // 7
        p.points.add(new Vec2(2.8,2.1)); // 8
        p.points.add(new Vec2(1.2,1.3)); // 9
        p.points.add(new Vec2(0.2,2.4)); // 10
        p.points.add(new Vec2(0.3,0.6)); // 11

        
        List<Polygon> parts = new PolygonConvexer(p).getParts();

        
        List<Polygon> expectedParts = new ArrayList<>();
        expectedParts.add(makeSubPolygon(p, 0,1,2));
        expectedParts.add(makeSubPolygon(p, 11,0,2));
        expectedParts.add(makeSubPolygon(p, 3,4,5));
        expectedParts.add(makeSubPolygon(p, 2,3,5,6));
        expectedParts.add(makeSubPolygon(p, 6,7,8,9));
        expectedParts.add(makeSubPolygon(p, 9,10,11));
        expectedParts.add(makeSubPolygon(p, 6,9,11));
        expectedParts.add(makeSubPolygon(p, 11,2,6));

        
        assertParts(parts, expectedParts);
    }

    Polygon makeSubPolygon(Polygon p, int ... vids) {
        Polygon res = new Polygon();
        for (int vid : vids) {
            res.points.add(p.points.get(vid));
        }
        return res;
    }

    void assertParts(List<Polygon> parts, List<Polygon> expectedParts) {
        Assert.assertEquals("Number of parts", expectedParts.size(), parts.size());
        for (int i = 0; i < parts.size(); ++i) {
            Polygon part = parts.get(i);
            Polygon expectedPart = expectedParts.get(i);
            Assert.assertEquals("Vertices in part "+Integer.toString(i), expectedPart.points.size(), part.points.size());
            for (int j = 0; j < part.points.size(); ++j) {
                Vec2 point = part.points.get(j);
                Vec2 expectedPoint = expectedPart.points.get(j);
                Assert.assertEquals("Vertex "+Integer.toString(j)+" in part "+Integer.toString(i), expectedPoint, point);
            }
        }
    }
}
