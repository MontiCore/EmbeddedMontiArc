public class DiscreteEvent {
    private MessageType type;
    private EEComponent target;
    private long requestTime;

    public DiscreteEvent() {}
    public DiscreteEvent(MessageType type, EEComponent target, long requestTime){
        this.target = target;
        this.type = type;
        this.requestTime = requestTime;
    }


    public MessageType getType() {
        return type;
    }

    public void setType(MessageType type) {
        this.type = type;
    }

    public EEComponent getTarget() {
        return target;
    }

    public long getRequestTime() {
        return requestTime;
    }
}
