/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;

import org.jfree.util.Log;
import simulation.EESimulator.EESimulator;

import java.time.Duration;
import java.time.Instant;
import java.util.Optional;
import java.util.PriorityQueue;

public class CAN extends Bus{

    private static final BusMessageComparatorIdDesc COMP_ID_DESC = new BusMessageComparatorIdDesc();

    /**
     * Header size in bits (Extended Frame Format since this is used in cars)
     */
    protected static final int HEADER_SIZE = 48;

    /**
     * Maximum payload size in bits
     */
    protected static final int MAX_PAYLOAD_SIZE = 64;

    /**
     * Trailer size in bits
     */
    protected static final int TRAILER_SIZE = 64;

    protected static final int MAX_MESSAGE_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + TRAILER_SIZE;

    private final CANOperationMode operationMode;

    private Instant currentTime;

    private PriorityQueue<BusMessage> messages = new PriorityQueue<>(COMP_ID_DESC);

    /**
     * BusMessage that is send in current (incompleted) frame. Transmissions can go over multiple calls of simulateUntil.
     */
    private Optional<BusMessage> partialMessage = Optional.empty();

    /**
     * The bits of the current frame that are already send. Transmissions can go over multiple calls of simulateUntil.
     */
    private int partialFrameBits = 0;

    protected CAN(EESimulator simulator, CANOperationMode operationMode) {
        super(simulator);
        this.currentTime = simulator.getSimulationTime();
        this.operationMode = operationMode;

    }

    /**
     * Return time to transmit one bit in nanoseconds.
     * @return time to transmit one bit in nanoseconds.
     */
    private long getBitTransmissionTime(){
        return (long) Math.ceil(1000000L / operationMode.getDataRate());
    }


    @Override
    public Instant getCurrentTime() {
        return currentTime;
    }

    @Override
    protected void simulateUntil(Instant endTime) {
        Duration timeDelta = Duration.between(currentTime, endTime);
        int toTransmit = Math.toIntExact(timeDelta.toNanos()/getBitTransmissionTime());
        //only forward time for bits actually send
        currentTime = currentTime.plusNanos(getBitTransmissionTime()*toTransmit);
        //finish last frame
        BusMessage msg = partialMessage.get();
        if(partialMessage.isPresent() && partialFrameBits != 0) {
            if (partialFrameBits < HEADER_SIZE) {
                partialFrameBits += Math.min(HEADER_SIZE - partialFrameBits, toTransmit);
                toTransmit -= Math.max(0, HEADER_SIZE - partialFrameBits);
            }
            int messageBytes = MAX_PAYLOAD_SIZE;
            if (partialFrameBits < HEADER_SIZE + MAX_PAYLOAD_SIZE) {
                partialFrameBits += Math.min(HEADER_SIZE + MAX_PAYLOAD_SIZE - partialFrameBits, toTransmit);
                messageBytes = Math.min(toTransmit, MAX_PAYLOAD_SIZE - partialFrameBits);
                messageBytes = msg.transmitBytes(messageBytes, operationMode.getBitErrorRate());
                toTransmit -= messageBytes;
                if (msg.isTransmitted()) {
                    registerMessageAtSimulator(msg);
                    msg = messages.poll();
                }
            }
            if (partialFrameBits < HEADER_SIZE + messageBytes + TRAILER_SIZE) {
                partialFrameBits += Math.min(HEADER_SIZE + messageBytes + TRAILER_SIZE - partialFrameBits, toTransmit);
                toTransmit -= Math.max(0, HEADER_SIZE + messageBytes + TRAILER_SIZE - partialFrameBits);
            }
        }
        //send full frames and incomplete last one
        while(toTransmit > HEADER_SIZE && msg != null) {
            partialFrameBits = toTransmit;
            toTransmit -= HEADER_SIZE;
            int messageBytes = Math.min(toTransmit, MAX_PAYLOAD_SIZE);
            messageBytes = msg.transmitBytes(messageBytes, operationMode.getBitErrorRate());
            toTransmit -= messageBytes;
            if (msg.isTransmitted()) {
                registerMessageAtSimulator(msg);
                msg = messages.poll();
            }
            toTransmit -= TRAILER_SIZE;
        }
        //0  < toTransmit  < HEADER_SIZE
        if(toTransmit > 0 && msg != null){
            partialFrameBits = toTransmit;
        }
        //all messages send
        if(msg == null){
            partialFrameBits = 0;
            partialMessage = Optional.empty();
        }
        else{
            partialMessage = Optional.of(msg);
        }
        
    }

    @Override
    protected Instant getNextFinishTime() {
        BusMessage msg = partialMessage.get();
        if(!partialMessage.isPresent()){
            msg = messages.poll();
        }
        if(msg == null){
            return currentTime;
        }
        int necessaryFrames = (msg.getRemainingBytes()*8) / MAX_PAYLOAD_SIZE;
        int remainingBits = (msg.getRemainingBytes()*8) % MAX_PAYLOAD_SIZE;
        int necessaryBits = necessaryFrames * MAX_MESSAGE_SIZE;
        necessaryBits += HEADER_SIZE + remainingBits + TRAILER_SIZE;
        return currentTime.plusNanos(calculateTransmissionTime(necessaryBits));
    }

    @Override
    protected void registerMessage(BusMessage msg) {
        if (!targetsByMessageId.containsKey(msg.getMessageID())) {
            Log.debug("Message has no target in Bus " + this.toString());
        }
        if (!msg.getTarget().getId().equals(this.getId())) {
            throw new IllegalArgumentException("Message with incorrect target @" + this.toString());
        }
        messages.offer(msg);
    }

    @Override
    protected boolean hasMessages() {
        return !messages.isEmpty() && !partialMessage.isPresent();
    }

    @Override
    public BusType getBusType() {
        return BusType.CAN;
    }

    protected long calculateTransmissionTime(long transmittedBytes) {
        long transmittedMikroBits = transmittedBytes * 1000000l;
        return (long) Math.ceil(transmittedMikroBits / operationMode.getDataRate());
    }
}
