package de.monticore.lang.monticar.generator.roscpp;

import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.roscpp.instructions.SubscribeInstruction;

public class ClockFrequencyExecutionStrategy implements ExecutionStrategy {
    private double deltaT;

    public ClockFrequencyExecutionStrategy(double deltaT) {
        this.deltaT = deltaT;
    }

    @Override
    public void generate(BluePrintCPP currentBluePrint) {
        Variable clockSub = addField(currentBluePrint, "_clockSubscriber", "ros::Subscriber");
        addField(currentBluePrint, "lastTick", "double");
        addInit(currentBluePrint, clockSub);
        addCheckTickMethod(currentBluePrint);
        addCallback(currentBluePrint);

    }

    private void addInit(BluePrintCPP currentBluePrint, Variable clockSub) {
        //TODO: dirty hack
        String compname = currentBluePrint.getName().replace(".h", "");
        Method constructor = currentBluePrint.getMethod(compname).orElse(null);
        if (constructor == null) throw new RuntimeException("Constructor must already be generated!");

        SubscribeInstruction subInstr = new SubscribeInstruction(compname, clockSub, "/clock", "_clockCallback");
        if (!constructor.getInstructions().contains(subInstr))
            constructor.addInstruction(subInstr);

    }

    private void addCheckTickMethod(BluePrintCPP currentBluePrint) {
        Method tickMethod = currentBluePrint.getMethod("tick").orElse(null);
        if (tickMethod == null) {
            throw new RuntimeException("Tick method must already be generated!");
        }

        Method method = new Method();
        method.setReturnTypeName("void");
        method.setName("tickIfNeeded");

        Variable parameter = new Variable();
        parameter.setTypeNameTargetLanguage("const rosgraph_msgs::Clock::ConstPtr&");
        parameter.setName("clockMsg");
        method.addParameter(parameter);

        String instrText = "double deltaT = " + deltaT + ";\n" +
                "\t\tdouble sinceLast = clockMsg->clock.toSec() - this->lastTick;\n" +
                "\t\tif(sinceLast >= deltaT){\n" +
                "\t\t\tif(sinceLast / deltaT > 2) ROS_WARN(\"Missed Ticks!\");\n" +
                "\t\t\tthis->lastTick = clockMsg->clock.toSec();\n" +
                "\t\t\ttick();\n" +
                "\t\t}\n";


        TargetCodeInstruction instr1 = new TargetCodeInstruction();
        instr1.setInstruction(instrText);
        method.addInstruction(instr1);
        currentBluePrint.addMethod(method);

    }

    private void addCallback(BluePrintCPP currentBluePrint) {
        Method method = currentBluePrint.getMethod("_clockCallback").orElse(null);
        if (method == null) {
            method = new Method();
            method.setName("_clockCallback");
            Variable variable = new Variable();
            variable.setName("msg");
            variable.setTypeNameTargetLanguage("const rosgraph_msgs::Clock::ConstPtr& msg");
            method.addParameter(variable);
            method.setReturnTypeName("void");

            currentBluePrint.addMethod(method);
        }

        TargetCodeInstruction tmpInstruction = new TargetCodeInstruction();
        tmpInstruction.setInstruction("tickIfNeeded(msg);");
        method.addInstruction(tmpInstruction);
    }

    Variable addField(BluePrintCPP currentBluePrint, String name, String type) {
        Variable var = currentBluePrint.getVariable(name).orElse(null);

        if (var == null) {
            var = new Variable();
            var.setTypeNameTargetLanguage(type);
            var.setName(name);
            currentBluePrint.addVariable(var);
        } else {
            if (!var.getVariableType().getTypeNameTargetLanguage().equals(type)) {
                throw new RuntimeException("Type mismatch: Field " + name + " has types " + type + " and " + var.getNameTargetLanguageFormat());
            }
        }

        return var;
    }

}
