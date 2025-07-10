/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import java.util.Random;

import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;

public class CANMessageTransmissionTest {
    public static Random rnd;

    @BeforeClass
    public static void setup() {
        rnd = new Random();
    }

    @Test
    public void testBitsizeComputation() throws EEMessageTypeException {
        testBitsizeComputation(CAN.MAX_PAYLOAD_SIZE_BYTES, 1,
                CAN.HEADER_SIZE_BITS + CAN.MAX_PAYLOAD_SIZE_BYTES * 8 + CAN.TRAILER_SIZE_BITS);
        testBitsizeComputation(2, 1, CAN.HEADER_SIZE_BITS + 2 * 8 + CAN.TRAILER_SIZE_BITS);
        testBitsizeComputation(CAN.MAX_PAYLOAD_SIZE_BYTES + 3, 2,
                CAN.FULL_FRAME + CAN.HEADER_SIZE_BITS + 3 * 8 + CAN.TRAILER_SIZE_BITS);
        testBitsizeComputation(CAN.MAX_PAYLOAD_SIZE_BYTES * 5, 5,
                CAN.FULL_FRAME * 4 + CAN.HEADER_SIZE_BITS + CAN.MAX_PAYLOAD_SIZE_BYTES * 8 + CAN.TRAILER_SIZE_BITS);
    }

    public void testBitsizeComputation(int msgSize, int expectedFrames, long expectedBits)
            throws EEMessageTypeException {
        CANMessageTransmission tr = new CANMessageTransmission(new Message(null, null, msgSize), rnd);
        Assert.assertEquals(expectedFrames, tr.getRequiredFrames());
        Assert.assertEquals(expectedBits, tr.getRequiredTotalBits());
    }
}