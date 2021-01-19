/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.loopSolver.daecpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMSymbolicVariableSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cmake.CMakeFindModule;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.ExecutionStepperHelper;
import de.monticore.lang.monticar.generator.cpp.GeneralHelperMethods;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.loopSolver.*;
import de.monticore.lang.monticar.semantics.loops.graph.EMAAtomicConnectorInstance;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.ComponentCall;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.EquationSystemFunction;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.ExplicitFunction;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.monticore.lang.monticar.semantics.setup.Delegate;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.Names;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class DAECPPSolveGenerator implements DAESolveGenerator, NonlinearSolveGenerator, ODESolveGenerator {

    public static final String timeName = "t";
    public static String xVarName = "x_DAE";

    @Override
    public void handleEquationSystem(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint,
                                     GeneratorCPP generatorCPP, List<String> includeStrings) {
        for (CMakeFindModule dependency : DAECPPOptions.getDependencies())
            generatorCPP.getCmakeConfig().addModuleDependency(dependency);
        ExecutionStepperHelper.setUsed();

        bluePrint.addAdditionalUserIncludeStrings("solver");
        bluePrint.addAdditionalUserIncludeStrings(ExecutionStepperHelper.FILENAME);

        Variable x = variable(xVarName, "daecpp::state_type", true, false);
        Variable mass = variable("mass", bluePrint.getName() + "_MassMatrix", true, true);
        Variable rhs = variable("rhs", bluePrint.getName() + "_RHS", true, true);
        Variable jac = variable("jac", "daecpp::Jacobian", true, false);
        Variable opt = variable("op" + timeName, "daecpp::SolverOptions", true, false);
        Variable solver = variable("solver", "daecpp::Solver", true, false);

        bluePrint.addVariable(x);
        bluePrint.addVariable(mass);
        bluePrint.addVariable(rhs);
        bluePrint.addVariable(jac);
        bluePrint.addVariable(opt);
        bluePrint.addVariable(solver);

        Method initOpt = new Method("initOptions", "static daecpp::SolverOptions");
        initOpt.setPublic(false);
        bluePrint.addMethod(initOpt);
        initOpt.addInstruction(new TargetCodeInstruction("daecpp::SolverOptions res;\n"));
        if (semiExplicitForm.getF().isEmpty())
            initOpt.addInstruction(new TargetCodeInstruction("res.time_stepping = 1;\n"));
        initOpt.addInstruction(new TargetCodeInstruction(String.format("res.atol = %s;\n", NumericSolverOptions.ATOL)));
        initOpt.addInstruction(new TargetCodeInstruction(String.format("res.dt_init = %s;\n", NumericSolverOptions.DT_SOLVER)));
        initOpt.addInstruction(new TargetCodeInstruction(String.format("res.dt_max = %s;\n", NumericSolverOptions.DT_SOLVER)));
        initOpt.addInstruction(new TargetCodeInstruction(String.format("res.verbosity = %s;\n", NumericSolverOptions.LEVEL_LOGGING)));
        initOpt.addInstruction(new TargetCodeInstruction("return res;\n"));

        Method constructor = new Method(bluePrint.getName(), "");
        bluePrint.setConstructor(constructor);
        String initials = Stream.concat(semiExplicitForm.getInitialValues().stream(),
                semiExplicitForm.getInitialGuesses().stream())
                .map(v -> v.getTextualRepresentation())
                .collect(Collectors.joining(","));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {%s},\n", x.getName(), initials)));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {},\n", mass.getName())));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {},\n", rhs.getName())));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {%s, %s},\n", jac.getName(), rhs.getName(), NumericSolverOptions.JTOL)));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {initOptions()},\n", opt.getName())));
        constructor.addInstruction(new TargetCodeInstruction(String.format("%s {%s, %s, %s, %s}",
                solver.getName(), rhs.getName(), jac.getName(), mass.getName(), opt.getName())));

        Method execute = bluePrint.getMethod("execute").orElse(null);
        execute.addInstruction(new TargetCodeInstruction(String.format("double %s = %s();\n", timeName, 
                ExecutionStepperHelper.CURRENTTIME_METHOD_NAME)));
        execute.addInstruction(new TargetCodeInstruction(String.format("solver(%s, %s);\n", xVarName, timeName)));

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

        for (FileContent fileContent : DAECPPEquationSystemGenerator.generateMassMatrix(semiExplicitForm, bluePrint.getName())) {
            generatorCPP.addFileContent(fileContent);
        }
    }

    @Override
    public void handleRHS(SemiExplicitForm semiExplicitForm, EMAMBluePrintCPP bluePrint, GeneratorCPP generatorCPP, List<String> includeStrings) {
        bluePrint.setName(bluePrint.getName() + " : public daecpp::RHS");

        bluePrint.addAdditionalUserIncludeStrings("solver");
        bluePrint.addAdditionalUserIncludeStrings(ExecutionStepperHelper.FILENAME);

        Method execute = bluePrint.getMethod("execute").orElse(new Method("execute", ""));

        if (bluePrint.getOriginalSymbol() instanceof RHSComponentInstanceSymbol) {
            EMAEquationSystem equationSystem = ((RHSComponentInstanceSymbol) bluePrint.getOriginalSymbol()).getEquationSystem();
            handleRHS(execute, semiExplicitForm, equationSystem.getIncomingPorts(), equationSystem.getOutgoingPorts(),
                    equationSystem.getConnectors());
        } else
            handleRHS(execute, semiExplicitForm, new HashSet<>(), new HashSet<>(), new HashSet<>());
    }



    private void handleRHS(Method method, SemiExplicitForm semiExplicitForm,
                           Collection<EMAPortInstanceSymbol> incomingPorts,
                           Collection<EMAPortInstanceSymbol> outgoingPorts,
                           Collection<EMAAtomicConnectorInstance> connectors) {
        method.setName("operator()");
        Variable par1 = new Variable("&x", "");
        par1.setVariableType(new VariableType("const daecpp::state_type", "const daecpp::state_type", ""));
        method.addParameter(par1);
        Variable par2 = new Variable("&f", "");
        par2.setVariableType(new VariableType("daecpp::state_type", "daecpp::state_type", ""));
        method.addParameter(par2);
        Variable par3 = new Variable(timeName, "");
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
            method.addInstruction(new TargetCodeInstruction(String.format("f[%s]=%s;\n", index++,
                    copy.getTextualRepresentation())));
        }
        for (EquationSystemFunction equationSystemFunction : semiExplicitForm.getG()) {
            if (equationSystemFunction instanceof ExplicitFunction) {
                EMAMEquationSymbol equation = ((ExplicitFunction) equationSystemFunction).getEquation();
                EMAMEquationSymbol emamEquationSymbol = Delegate.copyMathExpressionSymbol(equation);
                NameReplacer.replaceNames(emamEquationSymbol, n -> nameMapping.get(n));
                ExplicitFunction explicitFunction = new ExplicitFunction(emamEquationSymbol);
                method.addInstruction(new TargetCodeInstruction(String.format("f[%s]=%s;\n", index++,
                        explicitFunction.getAsRightSideExpression().getTextualRepresentation())));
            } else {
                // TODO
                equationSystemFunction.toString();
            }
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
