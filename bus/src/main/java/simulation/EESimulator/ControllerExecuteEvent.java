/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import java.time.Instant;

public class ControllerExecuteEvent extends EEDiscreteEvent {

    public ControllerExecuteEvent(Instant eventTime, EEComponent target) {
        super(EEDiscreteEventTypeEnum.CONTROLLER_EXECUTE_EVENT, eventTime, target);
    }

}
