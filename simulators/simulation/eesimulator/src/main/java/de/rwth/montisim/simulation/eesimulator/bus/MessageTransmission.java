/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus;

import java.util.Comparator;
import java.util.Random;

import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessagePriorityComparator;

public class MessageTransmission {

    public final Message msg;

    protected int transmittedBytes;

    protected boolean transmitted;

    protected boolean error;

    /**
     * Random number generator to determine a bit error
     */
    protected final Random bitError;

    public MessageTransmission(Message msg, Random rnd) {
        this.bitError = rnd;
        this.msg = msg;
        this.transmittedBytes = 0;
        this.transmitted = false;
        this.error = false;
    }

    public int getTransmittedBytes() {
        return transmittedBytes;
    }

    public boolean isTransmitted() {
        return transmitted;
    }

    public boolean isError() {
        return error;
    }

    public int getRemainingBytes() {
        return this.msg.msgLen - this.transmittedBytes;
    }

    /**
     * Transmit maximum number of bytes bytes of this message.
     *
     * @param bytes        Maximum number of bytes to be transmitted
     * @param bitErrorRate Rate of bit errors
     * @return Actual transmitted bytes
     */
    // public int transmitBytes(int bytes, double bitErrorRate) {
    // 	int res = -1;
    // 	if (bytes >= 0) {
    // 		if (this.msg.msgLen - this.transmittedBytes - bytes <= 0) {
    // 			transmitted = true;
    // 			res = this.msg.msgLen - this.transmittedBytes;
    // 			transmittedBytes = this.msg.msgLen;
    // 		} else {
    // 			transmittedBytes += bytes;
    // 			res = bytes;
    // 		}
    // 		// bound defines the probability of a bit error (1/bound)
    // 		int bound = Math.max(1, (int) ((1 / bitErrorRate) / (8 * res)));
    // 		if (this.bitError.nextInt(bound) == 0) {
    // 			this.error = true;
    // 		}
    // 	}
    // 	return res;
    // }

    /**
     * Creates a new message with target as destination and the finish time of this
     * event as event time.
     *
     * @param target Target of the new message
     * @return New BusMessageEvent with the updated target and event time.
     */
    // public BusMessageEvent forwardTo(EEComponent target) {
    // if(!this.transmitted) {
    // throw new IllegalArgumentException("Only transmitted messages can be
    // forwarded.");
    // }
    // else if(target == null){
    // throw new NullArgumentException();
    // }
    // else {
    // return new BusMessageEvent(this.message, this.messageLen, this.messageID,
    // this.finishTime, this.getTarget().getId(),
    // target, this.seenComponentIds);
    // }
    // }

    public static class MsgTransPriorityComp implements Comparator<MessageTransmission> {
        MessagePriorityComparator comp;

        public MsgTransPriorityComp(MessagePriorityComparator comp) {
            this.comp = comp;
        }

        public int compare(MessageTransmission a, MessageTransmission b) {
            return comp.compare(a.msg, b.msg);
        }
    }
}


