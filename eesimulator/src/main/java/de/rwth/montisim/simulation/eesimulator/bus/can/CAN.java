/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import java.time.Duration;
import java.time.Instant;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Random;

import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.MessageTransmission;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties.BusType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.message.MessagePriorityComparator;

/**
 * Models the transmission of BusMessageEvents of a CAN. Calculates the delay
 * accordingly.
 */
public class CAN extends Bus {
    // The Following is assuming the Extended frame format.
    // https://en.wikipedia.org/wiki/CAN_bus
    // NOTE: Not accounted right now: Bit stuffing, Errors, Error frames

    /**
     * Number of bits at the start of a frame, before the data payload. Header =
     * Start-of-frame + Identifier A + SRR + IDE + Identifier B + RTR + Reserved
     * Bits + DLC
     */
    public static final int HEADER_SIZE_BITS = 1 + 11 + 1 + 1 + 18 + 1 + 2 + 4;
    /**
     * Number of bits at the end of a frame, after the data payload. Trailer = CRC +
     * CRC delim + ACK + ACK delim + EOF
     */
    public static final int TRAILER_SIZE_BITS = 15 + 1 + 1 + 1 + 7;

    /**
     * Maximum payload size in bytes
     */
    public static final int MAX_PAYLOAD_SIZE_BYTES = 8;

    public static final int INTERFRAME_SPACE_BITS = 3;

    /**
     * Frame size with no payload and no inter-frame space.
     */
    public static final int PARTIAL_FRAME = CAN.HEADER_SIZE_BITS + CAN.TRAILER_SIZE_BITS;

    /**
     * Frame size with full payload and inter-frame bits
     */
    public static final int FULL_FRAME = CAN.INTERFRAME_SPACE_BITS + CAN.HEADER_SIZE_BITS
            + CAN.MAX_PAYLOAD_SIZE_BYTES * 8 + CAN.TRAILER_SIZE_BITS;

    public static final long HIGH_SPEED_CAN_BITRATE = 1_000_000;
    public static final long MEDIUM_SPEED_CAN_BITRATE = 500_000;
    public static final long LOW_SPEED_CAN_BITRATE = 125_000;

    public final transient CANProperties properties;

    private Instant startTime;
    // Time in the bus expressed in transmitted bits: Equivalent time in sec. is
    // bitTime/Bitrate. (bitTime "0" corresponds to startTime.)
    // Should always be synced to the start of a new transmission (after frame end +
    // inter-frame space). (Since frames are an "atom" in CAN transmission.)
    private long bitTime = 0;

    /**
     * Currently registered messages at this bus.
     */
    private PriorityQueue<CANMessageTransmission> messages; // TODO serialize

    private Optional<CANMessageTransmission> currentTransmission = Optional.empty(); // TODO serialize

    private final transient Random rnd = new Random();

    public CAN(CANProperties properties, EESystem eesystem, MessagePriorityComparator comp) {
        super(properties, eesystem);
        this.messages = new PriorityQueue<CANMessageTransmission>(new MessageTransmission.MsgTransPriorityComp(comp));
        this.properties = properties;
        this.startTime = eesystem.simulator.getSimulationTime();
    }

    @Override
    public BusType getBusType() {
        return BusType.CAN;
    }


    /**
     * Process a MessageReceiveEvent: register the message for transmission and handle current transmissions up to the event time.
     * If the new message changes the current next MessageReceiveEvent, it has to be invalidated.
     * Responsible of registering the next MessageReceiveEvent for the next message transmission completion.
     */
    @Override
    protected void receive(MessageReceiveEvent event) {
        MessageTransmission current = nextTransmission();
        messages.offer(new CANMessageTransmission(event.getMessage(), rnd));
        if (messages.peek() == current)
            return; // No change in the next event.

        // 2 remaining cases:
        // 1) There was no message being sent -> process the new message and create
        // an event for its transmission completion. (current == null)
        // 2) The new message might interrupts the current transmission (higher
        // priority)
        // if the current message uses multiple frames. If the current message is
        // a single frame message, the new message only overrides it if it is sent
        // before the identifier is transmitted. => Currently not the case (interrupts
        // only if sending at the same time with higher priority).

        // If there is a 'currentNextEvent':
        // - Find frame at which the transmission is interrupted.
        // - Update 'bitTime' to AFTER the frame = time where the next frame can be
        // sent.
        // Else simply advance time.
        long t = bitTimeFromInstant(event.getEventTime());
        // /!\ State based, currentTransmission must have an associated event.
        if (currentTransmission.isPresent()) { // 2)
            CANMessageTransmission msg = currentTransmission.get();
            long deltaBits = t - bitTime;
            int transmittedFrameCount = deltaBits == 0 ? 0 : (int) ((deltaBits - 1) / FULL_FRAME) + 1;
            int transmittedBits = transmittedFrameCount * FULL_FRAME;
            if (transmittedBits >= msg.getRemainingBits())
                return; // The current transmission is not interrupted.
            // Else interrupt transmission
            msg.transmitBits(transmittedBits);
            msg.event.get().invalid = true;
            currentTransmission = Optional.empty();

            bitTime += transmittedFrameCount * FULL_FRAME;
        } else { // 1)
            bitTime = t;
        }

        createNextSendEvent();
    }


    /**
     * Update the bus simulation to when the event's message is transmitted.
     * The message itself is already checked for validity and dispatched to its targets.
     * Responsible of registering the next MessageSendEvent if there are still messages to be transmitted.
     */
    @Override
    protected void messageSent(MessageSendEvent event) {
        if (!currentTransmission.isPresent() || currentTransmission.get().event.get() != event)
            throw new IllegalArgumentException("Unexpected MessageReceiveEvent");

        // Complete Transmission
        CANMessageTransmission msg = currentTransmission.get();
        bitTime += msg.getRemainingBits() + INTERFRAME_SPACE_BITS;
        msg.finished = true; // Tag the transmissions to remove them from the 'messages' (there is no
        // guarantee that they are at the front of the queue)
        currentTransmission = Optional.empty();

        createNextSendEvent();
    }

    // Assumes the current bitTime is synchronized to the last frame end
    // (+interspace) or to the current time.
    // NOTE: This model assumes that messages whose data doesn't fit into a single
    // frame
    // get transmitted across multiple frames by some transparent mechanism.
    private void createNextSendEvent() {
        CANMessageTransmission next = nextTransmission();
        if (next != null) {
            long finishTime = bitTime + next.getRemainingBits();
            MessageSendEvent evt = new MessageSendEvent(this, instantFromBitTime(finishTime), next.msg);
            next.event = Optional.of(evt);
            currentTransmission = Optional.of(next);
            this.eesystem.simulator.addEvent(evt);
        }
    }

    /**
     * Returns the next unfinished message transmission.
     */
    private CANMessageTransmission nextTransmission() {
        CANMessageTransmission c = messages.peek();
        while (c != null && c.finished) {
            messages.poll();
            c = messages.peek();
        }
        return c;
    }

    private Instant instantFromBitTime(long time) {
        return startTime.plus(Duration.ofNanos((time * Time.SECOND_TO_NANOSEC) / properties.bit_rate));
    }

    private long bitTimeFromInstant(Instant time) {
        long nanos = Time.nanosecondsFromDuration(Duration.between(startTime, time));
        return (nanos * properties.bit_rate) / Time.SECOND_TO_NANOSEC;
    }

}
