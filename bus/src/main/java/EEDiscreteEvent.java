import java.time.Instant;

public abstract class EEDiscreteEvent {

    private Instant eventTime;

    private EEComponent target;


    public EEDiscreteEvent(Instant eventTime, EEComponent target){
        this.target = target;
        this.eventTime = eventTime;
    }

    public Instant getEventTime() {
        return eventTime;
    }

    public void setEventTime(Instant eventTime) {
        this.eventTime = eventTime;
    }

    public EEComponent getTarget() {
        return target;
    }
}
