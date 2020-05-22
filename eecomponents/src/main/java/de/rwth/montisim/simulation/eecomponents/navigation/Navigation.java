package de.rwth.montisim.simulation.eecomponents.navigation;

import de.rwth.montisim.commons.dynamicinterface.ArrayType;
import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.dynamicinterface.ArrayType.Dimensionality;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class Navigation extends EEComponent {
    public static final String GPS_POS_MSG = "gps_pos";
    public static final String PUSH_TARGET_POS_MSG = "push_target_pos";
    public static final String POP_TARGET_POS_MSG = "push_target_pos";
    public static final String AT_TARGET_POS_MSG = "at_target_pos";
    public static final String CURRENT_TARGET_POS_MSG = "current_target_pos";
    // The following messages are sent regularly by the Navigation and give the one waypoint behind the vehicle and 4 after (targets)
    // In coordinate or local space (latlon - xy)
    // Might give less points if almost at target (or none if at target)
    public static final String TRAJECTORY_X_MSG = "trajectory_x";
    public static final String TRAJECTORY_Y_MSG = "trajectory_y";
    public static final String TRAJECTORY_LON_MSG = "trajectory_lon";
    public static final String TRAJECTORY_LAT_MSG = "trajectory_lat";

    final MessageInformation gpsPosMsg;
    
    final MessageInformation pushTargetPosMsg;
    final MessageInformation popTargetPosMsg;
    
    final MessageInformation atTargetPosMsg;
    final MessageInformation currentTargetPosMsg;

    final MessageInformation trajectoryXMsg;
    final MessageInformation trajectoryYMsg;
    final MessageInformation trajectoryLonMsg;
    final MessageInformation trajectoryLatMsg;

    public Navigation(EESimulator simulator, int priority) {
        super(simulator, "Navigation", priority);

        this.gpsPosMsg = addInput(GPS_POS_MSG, DataType.VEC2);

        this.pushTargetPosMsg = addInput(PUSH_TARGET_POS_MSG, DataType.VEC2, true);
        this.popTargetPosMsg = addInput(POP_TARGET_POS_MSG, DataType.EMPTY, true);
        
        this.atTargetPosMsg = addOutput(AT_TARGET_POS_MSG, DataType.BOOLEAN);
        this.currentTargetPosMsg = addOutput(CURRENT_TARGET_POS_MSG, DataType.VEC2);

        ArrayType at = new ArrayType(DataType.DOUBLE, Dimensionality.DYNAMIC, 5);
        this.trajectoryXMsg = addOutput(TRAJECTORY_X_MSG, at);
        this.trajectoryYMsg = addOutput(TRAJECTORY_Y_MSG, at);
        this.trajectoryLonMsg = addOutput(TRAJECTORY_LON_MSG, at);
        this.trajectoryLatMsg = addOutput(TRAJECTORY_LAT_MSG, at);
        // TODO Auto-generated constructor stub
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        // TODO Auto-generated method stub
        Message msg =  msgRecvEvent.getMessage();
        if (msg.msgId == gpsPosMsg.messageId){
            // Update position -> Check current routing
        } else if (msg.msgId == pushTargetPosMsg.messageId){
            // Add new position to routing stack -> compute new routing
        } else if (msg.msgId == popTargetPosMsg.messageId){
            // Remove target from routing stack -> compute new routing
        }
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.SERVICE;
    }
    
}