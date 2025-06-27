/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver.odeint;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.VariableType;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.*;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.loopSolver.NumericSolverOptions;
import de.monticore.lang.monticar.generator.cpp.loopSolver.ODESolveGenerator;
import de.monticore.lang.monticar.generator.cpp.loopSolver.RHSComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.ComponentCall;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.monticore.lang.monticar.semantics.setup.Delegate;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.Names;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ODEIntSolveGenerator implements ODESolveGenerator {

    public static String xVarName = "x_ODEINT";
    public static String timeName = "t_ODEINT";
    public static String stateTypeName = "state_type_ODEINT";

    @Override
    public void handleEquationSystem(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        for (CMakeFindModule dependency : OdeintOptions.getDependencies())
            generatorCPP.getCmakeConfig().addModuleDependency(dependency);
        ExecutionStepperHelper.setUsed();


        bluePrint.addAdditionalStandardInclude("iostream");
        bluePrint.addAdditionalStandardInclude("vector");
        bluePrint.addAdditionalStandardIncludeStringWithHPP("boost/numeric/odeint");
        bluePrint.addAdditionalUserIncludeStrings(ExecutionStepperHelper.FILENAME);


        bluePrint.addAdditionalTypeDefString(String.format("std::vector< double > %s", stateTypeName));
        bluePrint.addAdditionalTypeDefString(String.format("boost::numeric::odeint::runge_kutta_cash_karp54< %s > error_stepper_type", stateTypeName));
        bluePrint.addAdditionalTypeDefString("boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type");


        Variable lastTime = variable("lastTime", "double", true, false);
        Variable dtMax = variable("dt_max", "double", true, false);
        Variable x = variable(xVarName, stateTypeName, true, false);
        Variable rhs = variable("rhs", bluePrint.getName() + "_RHS", true, true);
        Variable stepper = variable("controlled_stepper", "controlled_stepper_type", true, false);
        bluePrint.addVariable(lastTime);
        bluePrint.addVariable(dtMax);
        bluePrint.addVariable(x);
        bluePrint.addVariable(rhs);
        bluePrint.addVariable(stepper);


        Method initStepper = new Method("initStepper", "static controlled_stepper_type");
        bluePrint.addMethod(initStepper);
        initStepper.setPublic(false);
        initStepper.addInstruction(new TargetCodeInstruction(
                String.format("controlled_stepper_type res(\n" +
                "boost::numeric::odeint::default_error_checker<\n" +
                "  double ,\n" +
                "  boost::numeric::odeint::range_algebra ,\n" +
                "  boost::numeric::odeint::default_operations>\n" +
                "  (%s, %s, 1.0, 1.0));\n", NumericSolverOptions.ATOL, NumericSolverOptions.RTOL)));
        initStepper.addInstruction(new TargetCodeInstruction("return res;\n"));


        Method constructor = new Method(bluePrint.getName(), "");
        bluePrint.setConstructor(constructor);
        String initials = Stream.concat(semiExplicitForm.getInitialValues().stream(),
                semiExplicitForm.getInitialGuesses().stream())
                .map(v -> v.getTextualRepresentation())
                .collect(Collectors.joining(","));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {%s},\n", x.getName(), initials)));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {},\n", rhs.getName())));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {%s()}", stepper.getName(), initStepper.getName())));


        Method execute = bluePrint.getMethod("execute").orElse(null);
        execute.addInstruction(new TargetCodeInstruction(String.format("double %s = %s();\n", timeName,
                ExecutionStepperHelper.CURRENTTIME_METHOD_NAME)));
        execute.addInstruction(new TargetCodeInstruction(String.format(
                "boost::numeric::odeint::integrate_adaptive(\n" +
                "  %s , %s, %s, %s, %s, %s);\n",
                stepper.getName(), rhs.getName(), x.getName(), lastTime.getName(), timeName, dtMax.getName()
                )));
        execute.addInstruction(new TargetCodeInstruction(String.format("%s = %s;\n",
                lastTime.getName(), timeName)));

        Map<String, Integer> nameMapping = new HashMap<>();
        int index = 0;
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getY())
            nameMapping.put(GeneralHelperMethods.
                    getTargetLanguageComponentVariableInstanceName(emamSymbolicVariableSymbol.getName()), index++);
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getZ())
            nameMapping.put(GeneralHelperMethods.
                    getTargetLanguageComponentVariableInstanceName(emamSymbolicVariableSymbol.getName()), index++);

        for (Variable variable : bluePrint.getVariables()) {
            if (variable.getAdditionalInformation().contains("outgoing")) {
                execute.addInstruction(new TargetCodeInstruction(String.format("%s = %s[%s];\n", variable.getName(),
                        xVarName, nameMapping.get(variable.getName()))));
            }
        }
    }

    @Override
    public void handleRHS(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        bluePrint.addAdditionalStandardIncludeStringWithHPP("boost/numeric/odeint");
        bluePrint.addAdditionalTypeDefString(String.format("std::vector< double > %s",stateTypeName));

        Method execute = bluePrint.getMethod("execute").orElse(new Method("execute", ""));

        if (bluePrint.getOriginalSymbol() instanceof RHSComponentInstanceSymbol) {
            EMAEquationSystem equationSystem = ((RHSComponentInstanceSymbol) bluePrint.getOriginalSymbol()).getEquationSystem();
            handleRHS(execute, bluePrint, semiExplicitForm, equationSystem.getIncomingPorts(), equationSystem.getOutgoingPorts(),
                    equationSystem.getConnectors());
        } else
            handleRHS(execute, bluePrint, semiExplicitForm, new HashSet<>(), new HashSet<>(), new HashSet<>());
    }



    public void handleRHS(Method method, EMAMBluePrintCPP bluePrint, SemiExplicitForm semiExplicitForm,
                          Collection<EMAPortInstanceSymbol> incomingPorts,
                          Collection<EMAPortInstanceSymbol> outgoingPorts,
                          Collection<EMAAtomicConnectorInstance> connectors) {
        method.setName("operator()");
        Variable par1 = new Variable("&x", "");
        par1.setVariableType(new VariableType(String.format("const %s",stateTypeName), String.format("const %s",stateTypeName), ""));
        method.addParameter(par1);
        Variable par2 = new Variable("&dxdt", "");
        par2.setVariableType(new VariableType(stateTypeName, stateTypeName, ""));
        method.addParameter(par2);
        Variable par3 = new Variable(String.format("/* %s */", timeName), "");
        par3.setVariableType(new VariableType("const double", "const double", ""));
        method.addParameter(par3);

        Set<EMAComponentInstanceSymbol> componentsToCall = semiExplicitForm.getG()
                .stream().filter(f -> f instanceof ComponentCall)
                .map(f -> ((ComponentCall) f).getComponent())
                .collect(Collectors.toSet());

        for (EMAPortInstanceSymbol incomingPort : incomingPorts) {
            if (!componentsToCall.contains(incomingPort.getComponentInstance())) continue;
            String componentName = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(
                    incomingPort.getComponentInstance().getFullName());
            String portName = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(
                    incomingPort.getFullName());
            method.addInstruction(new TargetCodeInstruction(String.format("%s = %s;\n",
                    Names.getQualifiedName(componentName, incomingPort.getName()),
                    portName)));
        }
        for (EMAConnectorInstanceSymbol connector : connectors) {
            if (outgoingPorts.contains(connector.getSourcePort()) &&
                    !incomingPorts.contains(connector.getTargetPort())) {
                // Connects to outside
            } else if (incomingPorts.contains(connector.getTargetPort())) {
                // Connectos from outside
            } else {
                // Interconnects
                if (!componentsToCall.contains(connector.getTargetPort().getComponentInstance())) continue;
                String target = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(
                        connector.getTargetPort().getComponentInstance().getFullName());
                target = Names.getQualifiedName(target, connector.getTargetPort().getName());
                int i = findIndex(connector.getSourcePort(), semiExplicitForm);
                method.addInstruction(new TargetCodeInstruction(String.format("%s = x[%s];\n",
                        target, i
                )));
                connector.toString();
            }
        }

        for (EMAComponentInstanceSymbol componentToCall : componentsToCall) {
            String call = GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(componentToCall.getFullName());
            method.addInstruction(new TargetCodeInstruction(String.format("%s.output();\n", call)));
        }

        Map<String, String> nameMapping = new HashMap<>();
        int index = 0;
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getY())
            nameMapping.put(emamSymbolicVariableSymbol.getName(), String.format("x[%s]", index++));
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getZ())
            nameMapping.put(emamSymbolicVariableSymbol.getName(), String.format("x[%s]", index++));
        for (EMAPortInstanceSymbol incomingPort : incomingPorts) {
            Optional<EMAPortInstanceSymbol> atomicSource =
                    connectors.stream()
                            .filter(c -> c.getTargetPort().equals(incomingPort))
                            .map(c -> c.getSourcePort())
                            .findFirst();
            if (atomicSource.isPresent()) {
                nameMapping.put(atomicSource.get().getFullName(),
                        GeneralHelperMethods.getTargetLanguageComponentVariableInstanceName(incomingPort.getFullName()));
            }
        }

        index = 0;
        for (MathExpressionSymbol mathExpressionSymbol : semiExplicitForm.getF()) {
            MathExpressionSymbol copy = Delegate.copyMathExpressionSymbol(mathExpressionSymbol);
            NameReplacer.replaceNames(copy, n -> nameMapping.get(n));
            MathFunctionFixer.fixMathFunctions(copy, bluePrint);
            String result = ExecuteMethodGenerator.generateExecuteCode(copy, bluePrint.getAdditionalUserIncludeStrings());
            method.addInstruction(new TargetCodeInstruction(String.format("dxdt[%s]=%s;\n", index++,
                    result)));
        }
    }

    private static int findIndex(EMAPortInstanceSymbol sourcePort, SemiExplicitForm semiExplicitForm) {
        int index = 0;

        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getY()) {
            if (emamSymbolicVariableSymbol.isPort() && emamSymbolicVariableSymbol.getPort().get().equals(sourcePort))
                return index;
            index++;
        }
        for (EMAMSymbolicVariableSymbol emamSymbolicVariableSymbol : semiExplicitForm.getZ()) {
            if (emamSymbolicVariableSymbol.isPort() && emamSymbolicVariableSymbol.getPort().get().equals(sourcePort))
                return index;
            index++;
        }

        return index;
    }

    private Variable variable(String name, String type, boolean isPrivate, boolean needsInclude) {
        Variable var = new Variable();
        var.setName(name);
        String includeName = needsInclude ? type : "";
        var.setVariableType(new VariableType(type, type, includeName));
        var.setPublic(!isPrivate);
        return var;
    }
}
