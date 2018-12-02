package de.monticore.lang.monticar.generator.roscpp.helper;

import de.monticore.lang.monticar.generator.roscpp.util.Instruction;
import de.monticore.lang.monticar.generator.roscpp.util.TargetCodeInstruction;

import java.util.ArrayList;
import java.util.List;

public class InitHelper {

    private InitHelper(){

    }

    public static List<Instruction> getInitInstructions(String classname, boolean isRos2){
        List<Instruction> result = new ArrayList<>();
        if (!isRos2){
            result.add(new TargetCodeInstruction("this->component = comp;"));
            result.add(new TargetCodeInstruction("char* tmp = strdup(\"\");"));
            result.add(new TargetCodeInstruction("int i = 0;"));
            result.add(new TargetCodeInstruction("ros::init(i, &tmp, \"" + classname + "_node\");"));
            result.add(new TargetCodeInstruction("ros::NodeHandle node_handle = ros::NodeHandle();"));
        }else{
            result.add(new TargetCodeInstruction("this->component = comp;"));
            result.add(new TargetCodeInstruction("char* tmp = strdup(\"\");"));
            result.add(new TargetCodeInstruction("int i = 0;"));
            result.add(new TargetCodeInstruction("rclcpp::init(i, &tmp);"));
        }
        return result;
    }

}
