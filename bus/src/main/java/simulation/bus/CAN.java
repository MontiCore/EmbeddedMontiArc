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
     * Header size in bytes (Extended Frame Format since this is used in cars)
     */
    protected static final int HEADER_SIZE = 6;

    /**
     * Maximum payload size in bytes
     */
    protected static final int MAX_PAYLOAD_SIZE = 8;

    /**
     * Trailer size in bits
     */
    protected static final int TRAILER_SIZE = 3;

    protected static final int MAX_MESSAGE_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + TRAILER_SIZE;

    private final CANOperationMode operationMode;

    private Instant currentTime;

    private PriorityQueue<BusMessage> messages = new PriorityQueue<>(COMP_ID_DESC);

    /**
     * BusMessage that is send in current (incompleted) frame. Transmissions can go over multiple calls of simulateUntil.
     */
    private Optional<BusMessage> partialMessage = Optional.empty();

    /**
     * The bytes of the current frame that are already send. Transmissions can go over multiple calls of simulateUntil.
     */
    private int partialFrameBytes = 0;

    private int partialFramePayloadBytes = 0;

    public CAN(EESimulator simulator, CANOperationMode operationMode) {
        super(simulator);
        this.currentTime = simulator.getSimulationTime();
        this.operationMode = operationMode;

    }

    @Override
    public Instant getCurrentTime() {
        return currentTime;
    }

    @Override
    protected void simulateUntil(Instant endTime) {
        Duration timeDelta = Duration.between(currentTime, endTime);
        long byteTransmissionTime = calculateTransmissionTime(1);
        int toTransmit = Math.toIntExact(timeDelta.toNanos()/byteTransmissionTime);
        toTransmit = finishLastFrame(toTransmit);
        //finish last frame
        BusMessage msg =  messages.peek();

        if(toTransmit > 0){
            partialFrameBytes = toTransmit;
            int messageBytes = 0;
            //send full frames and incomplete last one
            while(toTransmit > HEADER_SIZE && msg != null) {
                partialFrameBytes = toTransmit;
                toTransmit -= HEADER_SIZE;
                messageBytes = Math.min(toTransmit, MAX_PAYLOAD_SIZE);
                messageBytes = msg.transmitBytes(messageBytes, operationMode.getBitErrorRate());
                toTransmit -= messageBytes;
                if (msg.isTransmitted()) {
                    msg.setFinishTime(currentTime.plusNanos(calculateTransmissionTime(CAN.HEADER_SIZE + messageBytes + CAN.TRAILER_SIZE)));
                    registerMessageAtSimulator(msg);
                    messages.poll();
                    msg = messages.peek();
                }
                toTransmit -= TRAILER_SIZE;
                if(toTransmit >= 0){
                    currentTime = currentTime.plusNanos(calculateTransmissionTime(CAN.HEADER_SIZE + messageBytes + CAN.TRAILER_SIZE));
                }
                else {
                    currentTime = currentTime.plusNanos((calculateTransmissionTime(partialFrameBytes)));
                }
            }
            //add time for header
            if(toTransmit > 0){
                currentTime = currentTime.plusNanos((calculateTransmissionTime(toTransmit)));
            }

            //all messages send or packet completed
            if(msg == null ||
                    (partialFrameBytes < MAX_MESSAGE_SIZE && partialFrameBytes == CAN.HEADER_SIZE + messageBytes + CAN.TRAILER_SIZE) ) {
                partialFrameBytes = 0;
                partialFramePayloadBytes = 0;
                partialMessage = Optional.empty();
            }
            else {
                partialMessage = Optional.of(msg);
                partialFramePayloadBytes = messageBytes;
            }
        }
    }

    private int finishLastFrame(int toTransmit) {
        if(partialMessage.isPresent() && partialFrameBytes > 0 && partialFrameBytes < HEADER_SIZE + partialFramePayloadBytes + TRAILER_SIZE) {
            final int partialFrameBytesCopy = partialFrameBytes;
            BusMessage msg = partialMessage.get();
            if (partialFrameBytes < HEADER_SIZE) {
                int headerBytes = HEADER_SIZE - partialFrameBytes;
                partialFrameBytes += Math.min(headerBytes, toTransmit);
                toTransmit -= headerBytes;
            }
            if (partialFrameBytes < HEADER_SIZE + MAX_PAYLOAD_SIZE) {
                int messageBytes = Math.min(HEADER_SIZE + MAX_PAYLOAD_SIZE - partialFrameBytes, toTransmit);
                messageBytes = msg.transmitBytes(messageBytes, operationMode.getBitErrorRate());
                toTransmit -= messageBytes;
                partialFramePayloadBytes += messageBytes;
                partialFrameBytes += messageBytes;
                if (msg.isTransmitted()) {
                    msg.setFinishTime(currentTime.plusNanos(calculateTransmissionTime(partialFrameBytes - partialFrameBytesCopy)));
                    registerMessageAtSimulator(msg);
                    messages.poll();
                }
            }
            if (partialFrameBytes < HEADER_SIZE + partialFramePayloadBytes + TRAILER_SIZE) {
                int trailerBytes = HEADER_SIZE + partialFramePayloadBytes + TRAILER_SIZE - partialFrameBytes;
                partialFrameBytes += Math.min(trailerBytes, toTransmit);
                toTransmit -= trailerBytes;
            }
            //set time
            currentTime = currentTime.plusNanos(calculateTransmissionTime(partialFrameBytes - partialFrameBytesCopy));
            //packet completed
            if(partialFrameBytes == CAN.HEADER_SIZE + partialFramePayloadBytes + CAN.TRAILER_SIZE){
                partialFrameBytes = 0;
                partialFramePayloadBytes = 0;
                partialMessage = Optional.empty();
            }
            if(toTransmit > 0 && (partialFrameBytes != 0 || partialFramePayloadBytes != 0 || partialMessage.isPresent())){
                throw new IllegalStateException("to transmit can not be greater than 0 while packet not completed");
            }
        }
        return toTransmit;
    }

    @Override
    protected Instant getNextFinishTime() {
        BusMessage msg;

        int necessaryBytes = 0;
        int partialPayload = 0;
        if(!partialMessage.isPresent()){
            msg = messages.peek();
        }
        else{
            msg = partialMessage.get();
            if (partialFrameBytes < HEADER_SIZE) {
                necessaryBytes += HEADER_SIZE - partialFrameBytes;
            }
            if (partialFrameBytes + necessaryBytes < HEADER_SIZE + MAX_PAYLOAD_SIZE) {
                partialPayload = Math.min(HEADER_SIZE + MAX_PAYLOAD_SIZE - partialFrameBytes, msg.getRemainingBytes());
                necessaryBytes += partialPayload;
            }
            if (partialFrameBytes + necessaryBytes < HEADER_SIZE + partialFramePayloadBytes + partialPayload + TRAILER_SIZE) {
                necessaryBytes += HEADER_SIZE + partialFramePayloadBytes + TRAILER_SIZE - partialFrameBytes;
            }
        }
        if(msg == null){
            return currentTime;
        }
        int remainingMsgBytes = msg.getRemainingBytes() - partialPayload;
        int necessaryFrames = remainingMsgBytes/ MAX_PAYLOAD_SIZE;
        int remainingBytes = remainingMsgBytes % MAX_PAYLOAD_SIZE;
        necessaryBytes += necessaryFrames * MAX_MESSAGE_SIZE;
        if(remainingBytes != 0){
            necessaryBytes += HEADER_SIZE + remainingBytes + TRAILER_SIZE;
        }
        return currentTime.plusNanos(calculateTransmissionTime(necessaryBytes));
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
    protected OperationMode getOperationMode() {
        return operationMode;
    }

    @Override
    public BusType getBusType() {
        return BusType.CAN;
    }

}
