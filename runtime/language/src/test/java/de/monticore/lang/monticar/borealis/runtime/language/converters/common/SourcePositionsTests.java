package de.monticore.lang.monticar.borealis.runtime.language.converters.common;

import de.se_rwth.commons.SourcePosition;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class SourcePositionsTests {
    final SourcePosition start = new SourcePosition(10, 40);
    final SourcePosition end = new SourcePosition(20, 10);
    final SourcePositions sourcePositions = new SourcePositions(start, end);

    @Test
    void testEquals() {
        /*
         * Equal Object
         */
        SourcePosition validStart = new SourcePosition(10, 40);
        SourcePosition validEnd = new SourcePosition(20, 10);
        SourcePositions validPeer = new SourcePositions(validStart, validEnd);

        /*
         * Unequal Object
         */
        SourcePosition invalidStart = new SourcePosition(20, 85);
        SourcePosition invalidEnd = new SourcePosition(30, 25);
        SourcePositions invalidPeer = new SourcePositions(invalidStart, invalidEnd);

        assertAll(
            "Object Equality",
            () -> assertEquals(sourcePositions, validPeer, "The objects should be marked as equal."),
            () -> assertNotEquals(sourcePositions, invalidPeer, "The objects should not be marked as equal.")
        );
    }
}
