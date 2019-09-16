package simulation.EESimulator;

import java.time.Instant;

public class ControllerExecuteEvent extends EEDiscreteEvent {

    public ControllerExecuteEvent(Instant eventTime, EEComponent target) {
        super(EEDiscreteEventTypeEnum.CONTROLLER_EXECUTE_EVENT, eventTime, target);
    }

}
