/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import java.util.Optional;
import java.util.Random;

import de.rwth.montisim.simulation.eesimulator.bus.MessageTransmission;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;

/**
 * Doesn't update getRemainingBytes(), isTransmitted() and getRemainingBytes():
 * Use Bits variants (that are transmission oriented, not payload oriented).
 */
public class CANMessageTransmission extends MessageTransmission {

    int requiredFrames;
    long requiredTotalBits;
    long transmittedTotalBits;
    Optional<MessageSendEvent> event;
    boolean finished;

    public CANMessageTransmission(Message msg, Random rnd) {
        super(msg, rnd);
        this.finished = false;
        this.event = Optional.empty();
        int bytesToTransmit = msg.msgLen;
        this.transmittedTotalBits = 0;
        this.requiredFrames = bytesToTransmit == 0 ? 1 : ((bytesToTransmit - 1) / CAN.MAX_PAYLOAD_SIZE_BYTES) + 1;
        int lastFramePayload = bytesToTransmit == 0 ? 0 : ((bytesToTransmit - 1) % CAN.MAX_PAYLOAD_SIZE_BYTES) + 1;

        // For last frame
        this.requiredTotalBits =
                CAN.PARTIAL_FRAME +
                        lastFramePayload * 8;

        // If multiple frames => INTERFRAME_SPACE_BITS
        if (this.requiredFrames > 1) {
            this.requiredTotalBits += (this.requiredFrames - 1) * CAN.FULL_FRAME;
        }
    }

    public int getRequiredFrames() {
        return requiredFrames;
    }

    public long getRequiredTotalBits() {
        return requiredTotalBits;
    }

    public void transmitBits(long bits) {
        transmittedTotalBits += bits;
        if (transmittedTotalBits > requiredTotalBits) transmittedTotalBits = requiredTotalBits;
    }

    public long getRemainingBits() {
        return requiredTotalBits - transmittedTotalBits;
    }
}