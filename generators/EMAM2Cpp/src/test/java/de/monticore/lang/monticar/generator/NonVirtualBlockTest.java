/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.monticar.generator.order.NonVirtualBlock;
import org.junit.Test;

import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

/**
 * This class is based on the NonVirtualBlockTest from the montiarc-executionOrder project
 * but uses the NonVirtualBlock from this project
 */
public class NonVirtualBlockTest {

    @Test
    public void testHashCodeAndEquals() {
        //set variables
        int s = 0;
        int s2 = 2;
        int b = 5;
        int b2 = 0;
        Optional<Integer> x = Optional.empty();
        Integer x2 = 4;
        Optional<Integer> y = Optional.empty();
        Integer y2 = 0;

        //initialize NonVirtualBlocks
        NonVirtualBlock block1 = new NonVirtualBlock(s, b);
        NonVirtualBlock block2 = new NonVirtualBlock(s, b);
        NonVirtualBlock block3 = new NonVirtualBlock(s2, b);
        NonVirtualBlock block4 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block5 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block6 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block7 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block8 = new NonVirtualBlock(s2, b2);

        //set parameters
        block5.setX(x);
        block5.setY(y);

        block6.setX(Optional.of(x2));
        block6.setY(Optional.of(y2));

        block7.setX(Optional.of(x2));

        block8.setX(Optional.of(y2));

        //test equals() and hashCode() without x and y
        assertEquals(true, block1.equals(block1));
        assertEquals(block1.hashCode(), block1.hashCode());
        assertEquals(true, block1.equals(block2));
        assertEquals(block1.hashCode(), block2.hashCode());
        assertEquals(false, block1.equals(block3));
        assertNotEquals(block1.hashCode(), block3.hashCode());
        assertEquals(false, block2.equals(block3));
        assertNotEquals(block2.hashCode(), block3.hashCode());
        assertEquals(false, block1.equals(b));

        //test equals() and hashCode() with x and y
        assertEquals(true, block4.equals(block5));
        assertEquals(block4.hashCode(), block5.hashCode());
        assertEquals(false, block4.equals(block6));
        assertNotEquals(block4.hashCode(), block6.hashCode());
        assertEquals(false, block5.equals(block6));
        assertNotEquals(block5.hashCode(), block6.hashCode());
    }

    @Test
    public void testToString() {
        //set variables
        int s = 0;
        int s2 = 2;
        int b = 5;
        int b2 = 0;
        Integer x2 = 4;
        Integer y2 = 0;

        //initialize NonVirtualBlocks
        NonVirtualBlock block6 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block7 = new NonVirtualBlock(s, b);
        NonVirtualBlock block8 = new NonVirtualBlock(s2, b2);

        //set parameters
        block6.setX(Optional.of(x2));
        block6.setY(Optional.of(y2));

        block7.setX(Optional.of(x2));

        block8.setY(Optional.of(y2));

        //test toString()
        assertEquals("0:0{4,0}", block6.toString());
        assertEquals("0:5{4}", block7.toString());
        assertEquals("2:0{0}", block8.toString());
    }

    @Test
    public void testCompareTo() {
        //set variables
        int s = 0;
        int s2 = 2;
        int b = 5;
        int b2 = 0;
        Integer x2 = 4;
        Integer y2 = 0;

        //initialize NonVirtualBlocks
        NonVirtualBlock block1 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block2 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block3 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block4 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block5 = new NonVirtualBlock(s, b2);
        NonVirtualBlock block6 = new NonVirtualBlock(s2, b2);

        //set parameters
        block1.setX(Optional.of(x2));
        block1.setY(Optional.of(y2));

        block2.setX(Optional.of(x2));
        block2.setY(Optional.of(s2));

        block3.setX(Optional.of(b));
        block3.setY(Optional.of(b));

        block4.setY(Optional.of(b));

        block5.setX(Optional.of(b));

        //test compareTo()
        assertEquals(-1, block1.compareTo(block2));
        assertEquals(1, block2.compareTo(block1));
        assertEquals(0, block1.compareTo(block1));
        assertEquals(1, block3.compareTo(block2));
        assertEquals(-1, block2.compareTo(block3));
        assertEquals(-1, block4.compareTo(block5));
        assertEquals(1, block5.compareTo(block4));
        assertEquals(1, block3.compareTo(block4));
        assertEquals(1, block3.compareTo(block5));
        assertEquals(-1, block5.compareTo(block3));
        assertEquals(-1, block4.compareTo(block3));
        assertEquals(-1, block1.compareTo(block6));
        assertEquals(1, block6.compareTo(block1));
    }

}
