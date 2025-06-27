/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.semantics.helper.EMAPropertiesHelper;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.loops.symbols.EMAEquationSystem;
import de.monticore.lang.monticar.semantics.setup.Delegate;
import de.monticore.lang.monticar.semantics.util.math.MathHelper;
import de.monticore.lang.monticar.semantics.util.math.NameReplacer;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

public class SpecificationConverter {

    public static Optional<EMAMSpecificationSymbol> convert(EMAEquationSystem system, EMAComponentInstanceSymbol component) {
        Optional<EMAMSpecificationSymbol> specification = convert(component);
        if (!specification.isPresent()) return specification;

        Map<String, String> nameMapping = getNameMapping(component, system);
        specification.get().setVariables(specification.get().getVariables().stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet()));
        specification.get().setEquations(specification.get().getEquations().stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet()));
        specification.get().setInitialValues(specification.get().getInitialValues().stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet()));
        specification.get().setInitialGuesses(specification.get().getInitialGuesses().stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet()));
        return specification;
    }


    public static Optional<EMAMSpecificationSymbol> convert(EMAComponentInstanceSymbol component) {
        if (!EMAPropertiesHelper.isAtomic(component)) return Optional.empty();
        Optional<EMAMSpecificationSymbol> result = getSpecificationFromMath(component);
        if (!result.isPresent()) result = calculateSpecificationFromAssignments(component);
        if (result.isPresent())
            return Optional.ofNullable(MathFunctionFixer.replaceInnerArrayByIndividualPorts(result.get()));
        return Optional.empty();
    }

    public static Optional<EMAMSpecificationSymbol> getSpecificationFromMath(EMAComponentInstanceSymbol component) {
        Collection<MathStatementsSymbol> symbols = component.getSpannedScope().resolveLocally(MathStatementsSymbol.KIND);
        if (symbols.size() != 1) return Optional.empty();
        MathStatementsSymbol mathStatementsSymbol = symbols.stream().findFirst().get();
        Optional<EMAMSpecificationSymbol> first = mathStatementsSymbol.getMathExpressionSymbols()
                .stream()
                .filter(e -> e instanceof EMAMSpecificationSymbol)
                .map(s -> (EMAMSpecificationSymbol) s)
                .findFirst();

        if (!first.isPresent())
            return Optional.empty();

        EMAMSpecificationSymbol specification = first.get();
        return Optional.of(specification);
    }


    private static Optional<EMAMSpecificationSymbol> calculateSpecificationFromAssignments(EMAComponentInstanceSymbol component) {
        Collection<EMAMSymbolicVariableSymbol> variables = getOutgoingPortsAsVariables(component);
        Collection<EMAMInitialValueSymbol> initialValues = getPortInitialValues(component);
        Collection<EMAMInitialGuessSymbol> initialGuesses = getPortInitialGuesses(component);

        if (component.getSpannedScope().resolveLocally(MathStatementsSymbol.KIND).size() != 1)
            return Optional.empty();

        Collection<EMAMEquationSymbol> equations = calculateEquationsFromAssignments(component, variables);

        return Optional.of(new EMAMSpecificationSymbolSynth(variables, equations, initialValues, initialGuesses));
    }

    public static Collection<EMAMSymbolicVariableSymbol> getOutgoingPortsAsVariables(EMAEquationSystem system,
                                                                                     EMAComponentInstanceSymbol component) {
        Map<String, String> nameMapping = getNameMapping(component, system);
        return getOutgoingPortsAsVariables(component).stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet());
    }

    public static Collection<EMAMSymbolicVariableSymbol> getOutgoingPortsAsVariables(EMAComponentInstanceSymbol component) {
        Collection<EMAMSymbolicVariableSymbol> variables = new HashSet<>();
        for (EMAPortInstanceSymbol port : component.getOutgoingPortInstances()) {
            // TODO maybe array
            EMAMSymbolicVariableSymbol var = new EMAMSymbolicVariableSymbol(port.getName());
            var.setPort(port);
            variables.add(var);
        }
        return variables;
    }

    public static Collection<EMAMSymbolicVariableSymbol> getIncomingInformationAsVariables(EMAEquationSystem system) {
        Collection<EMAMSymbolicVariableSymbol> result = new HashSet<>();
        system.getIncomingPorts().stream()
                .map(i -> system.getAtomicSourceOf(i))
                .filter(source -> source.isPresent())
                .map(source -> source.get())
                .forEach(source -> {
                    Optional<EMAMSymbolicVariableSymbol> var = getOutgoingPortsAsVariables(system, source.getComponentInstance())
                            .stream().filter(v -> v.isPort())
                            .filter(v -> v.getPort().get() == source)
                            .findFirst();
                    if (var.isPresent())
                        result.add(var.get());
                });
        return result;
    }

    public static Collection<EMAMSymbolicVariableSymbol> getIncomingInformationAsVariables(EMAEquationSystem system,
                                                                                           EMAComponentInstanceSymbol component) {
        Map<String, String> nameMapping = getNameMapping(component, system);
        return getIncomingInformationAsVariables(component).stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet());
    }

    public static Collection<EMAMSymbolicVariableSymbol> getIncomingInformationAsVariables(EMAComponentInstanceSymbol component) {
        Collection<EMAMSymbolicVariableSymbol> variables = new HashSet<>();
        for (EMAPortInstanceSymbol port : component.getIncomingPortInstances()) {
            // TODO maybe array
            EMAMSymbolicVariableSymbol var = new EMAMSymbolicVariableSymbol(port.getName());
            var.setPort(port);
            variables.add(var);
        }
        return variables;
    }

    public static Collection<EMAMInitialValueSymbol> getPortInitialValues(EMAComponentInstanceSymbol component) {
        Collection<EMAMInitialValueSymbol> initialValues = new HashSet<>();
        for (EMAPortInstanceSymbol port : component.getOutgoingPortInstances()) {
            if (port.isInitialValuePresent())
                initialValues.add(convertInitialValue(port));
        }
        return initialValues;
    }

    public static Collection<EMAMInitialGuessSymbol> getPortInitialGuesses(EMAComponentInstanceSymbol component) {
        Collection<EMAMInitialGuessSymbol> initialGuesses = new HashSet<>();
        for (EMAPortInstanceSymbol port : component.getOutgoingPortInstances()) {
            if (port.isInitialGuessPresent())
                initialGuesses.add(convertInitialGuess(port));
        }
        return initialGuesses;
    }

    public static Collection<EMAMEquationSymbol> calculateEquationsFromAssignments(EMAComponentInstanceSymbol component,
                                                                                   Collection<EMAMSymbolicVariableSymbol> variables) {
        Collection<MathStatementsSymbol> symbols = component.getSpannedScope().resolveLocally(MathStatementsSymbol.KIND);
        if (symbols.size() != 1)
            return new HashSet<>();

        ExpressionConverter expressionConverter = new ExpressionConverter();
        expressionConverter.variables.addAll(variables);

        for (MathExpressionSymbol expressionSymbol : symbols.stream().findFirst().get().getMathExpressionSymbols()) {
            MathExpressionSymbol replaced = PredefinedExpressionReplacer.replace(expressionSymbol);
            if (replaced.isAssignmentExpression()) {
                expressionConverter.handleExpression((MathAssignmentExpressionSymbol) replaced);
            } else if (replaced instanceof MathForLoopExpressionSymbol) {
                // TODO
                MathForLoopExpressionSymbol forLoopExpressionSymbol = (MathForLoopExpressionSymbol) replaced;
                forLoopExpressionSymbol.toString();
            } else if (replaced instanceof MathValueSymbol) {
                // TODO

            } else {
                Log.error("0x654987 not yet supported");
            }
        }
        variables.addAll(expressionConverter.variables);
        return expressionConverter.system;
    }


    private static Map<EMAEquationSystem, EMAMSpecificationSymbol> emaeqs2SpecMap = new HashMap<>();
    private static Map<EMAComponentInstanceSymbol, EMAMSpecificationSymbol> emacis2SpecMap = new HashMap<>();

    public static Optional<EMAMSpecificationSymbol> convert(EMAEquationSystem system) {
        if (emaeqs2SpecMap.containsKey(system))
            return Optional.ofNullable(emaeqs2SpecMap.get(system));

        Collection<EMAMSymbolicVariableSymbol> variables = new HashSet<>();
        Collection<EMAMEquationSymbol> equations = new HashSet<>();
        Collection<EMAMInitialValueSymbol> initialValues = new HashSet<>();
        Collection<EMAMInitialGuessSymbol> initialGuesses = new HashSet<>();

        for (EMAComponentInstanceSymbol component : system.getComponentInstanceSymbols()) {
            EMAMSpecificationSymbol specificationSymbol;
            if (emacis2SpecMap.containsKey(component))
                specificationSymbol = emacis2SpecMap.get(component);
            else {
                Optional<EMAMSpecificationSymbol> convert = convert(component);
                if (!convert.isPresent()) return Optional.empty();
                else specificationSymbol = convert.get();
            }

            // replace portNames by fullNames
            Map<String, String> nameMapping = getNameMapping(component, system);

            for (EMAMEquationSymbol equation : specificationSymbol.getEquations())
                equations.add(replaceName(equation, nameMapping));

            for (EMAMSymbolicVariableSymbol variable : specificationSymbol.getVariables())
                variables.add(replaceName(variable, nameMapping));

            for (EMAMInitialGuessSymbol initialGuess : specificationSymbol.getInitialGuesses())
                initialGuesses.add(replaceName(initialGuess, nameMapping));

            for (EMAMInitialValueSymbol initialValue : specificationSymbol.getInitialValues())
                initialValues.add(replaceName(initialValue, nameMapping));
        }

        return Optional.of(new EMAMSpecificationSymbolSynth(variables, equations, initialValues, initialGuesses));
    }

    public static Collection<EMAMSymbolicVariableSymbol> getVariables(EMAEquationSystem system) {
        Collection<EMAMSymbolicVariableSymbol> variables = new HashSet<>();
        for (EMAComponentInstanceSymbol component : system.getComponentInstanceSymbols())
            variables.addAll(getVariables(system, component));
        return variables;
    }

    public static Collection<EMAMEquationSymbol> getEquations(EMAEquationSystem system) {
        Collection<EMAMEquationSymbol> equations = new HashSet<>();
        for (EMAComponentInstanceSymbol component : system.getComponentInstanceSymbols()) {
            Collection<EMAMEquationSymbol> componentEquations = getEquations(system, component);
            if (componentEquations.isEmpty())
                return new HashSet<>();
            equations.addAll(componentEquations);
        }
        return equations;
    }

    public static Collection<EMAMInitialGuessSymbol> getInitialGuesses(EMAEquationSystem system) {
        Collection<EMAMInitialGuessSymbol> initialGuesses = new HashSet<>();
        for (EMAComponentInstanceSymbol component : system.getComponentInstanceSymbols())
            initialGuesses.addAll(getInitialGuesses(system, component));
        return initialGuesses;
    }

    public static Collection<EMAMInitialValueSymbol> getInitialValues(EMAEquationSystem system) {
        Collection<EMAMInitialValueSymbol> initialValues = new HashSet<>();
        for (EMAComponentInstanceSymbol component : system.getComponentInstanceSymbols())
            initialValues.addAll(getInitialValues(system, component));
        return initialValues;
    }


    public static Collection<EMAMSymbolicVariableSymbol> getVariables(EMAComponentInstanceSymbol component) {
        Optional<EMAMSpecificationSymbol> spec = getSpecificationFromMath(component);
        if (spec.isPresent()) return spec.get().getVariables();
        else
            return getOutgoingPortsAsVariables(component); // Maybe should add some of the assignments from the assignments
    }

    public static Collection<EMAMEquationSymbol> getEquations(EMAComponentInstanceSymbol component) {
        Optional<EMAMSpecificationSymbol> spec = getSpecificationFromMath(component);
        if (spec.isPresent()) return spec.get().getEquations();
        else return calculateEquationsFromAssignments(component, getOutgoingPortsAsVariables(component));
    }

    public static Collection<EMAMInitialGuessSymbol> getInitialGuesses(EMAComponentInstanceSymbol component) {
        Optional<EMAMSpecificationSymbol> spec = getSpecificationFromMath(component);
        if (spec.isPresent()) return spec.get().getInitialGuesses();
        else return getPortInitialGuesses(component); // Maybe should add some of the assignments from the assignments
    }

    public static Collection<EMAMInitialValueSymbol> getInitialValues(EMAComponentInstanceSymbol component) {
        Optional<EMAMSpecificationSymbol> spec = getSpecificationFromMath(component);
        if (spec.isPresent()) return spec.get().getInitialValues();
        else return getPortInitialValues(component); // Maybe should add some of the assignments from the assignments
    }


    public static Collection<EMAMSymbolicVariableSymbol> getVariables(EMAEquationSystem system, EMAComponentInstanceSymbol component) {
        Map<String, String> nameMapping = getNameMapping(component, system);
        return getVariables(component).stream()
                .map(v -> replaceName(v, nameMapping))
                .collect(Collectors.toSet());
    }

    public static Collection<EMAMEquationSymbol> getEquations(EMAEquationSystem system, EMAComponentInstanceSymbol component) {
        Map<String, String> nameMapping = getNameMapping(component, system);
        return getEquations(component).stream()
                .map(e -> replaceName(e, nameMapping))
                .collect(Collectors.toSet());
    }

    public static Collection<EMAMInitialGuessSymbol> getInitialGuesses(EMAEquationSystem system, EMAComponentInstanceSymbol component) {
        Map<String, String> nameMapping = getNameMapping(component, system);
        return getInitialGuesses(component).stream()
                .map(i -> replaceName(i, nameMapping))
                .collect(Collectors.toSet());
    }

    public static Collection<EMAMInitialValueSymbol> getInitialValues(EMAEquationSystem system, EMAComponentInstanceSymbol component) {
        Map<String, String> nameMapping = getNameMapping(component, system);
        return getInitialValues(component).stream()
                .map(i -> replaceName(i, nameMapping))
                .collect(Collectors.toSet());
    }

    private static Map<String, String> getNameMapping(EMAComponentInstanceSymbol component, EMAEquationSystem system) {
        // replace portNames by fullNames
        Map<String, String> nameMapping = new HashMap<>();

        component.getOutgoingPortInstances()
                .stream()
                .forEach(port -> nameMapping.put(replacePortBrackets(port.getName()),
                        replacePortBrackets(NameHelper.calculateFullQualifiedNameOf(port))));

        for (EMAPortInstanceSymbol inport : component.getIncomingPortInstances()) {
            Optional<EMAPortInstanceSymbol> source = system.getAtomicSourceOf(inport);
            if (!source.isPresent()) {
//                Log.warn(String.format("0x1544231 no source port for input port %s",
//                        NameHelper.calculateFullQualifiedNameOf(inport)));
                source = Optional.of(inport);
            }

            String sourcePortFullName = replacePortBrackets(NameHelper.calculateFullQualifiedNameOf(source.get()));
            nameMapping.put(replacePortBrackets(inport.getName()), sourcePortFullName);
        }

        String componentFullName = NameHelper.calculateFullQualifiedNameOf(component);
        for (EMAMSymbolicVariableSymbol variable : getOutgoingPortsAsVariables(component))
            if (!nameMapping.containsKey(variable.getName()))
                nameMapping.put(variable.getName(), Names.getQualifiedName(componentFullName, variable.getName()));

        return nameMapping;
    }

    private static String replacePortBrackets(String port) {
        return port.replace("[", "(").replace("]", ")");
    }


    private static <T extends MathExpressionSymbol> T replaceName(T mathExpressionSymbol, Map<String, String> nameMapping) {
        Map<String, String> newNameMapping = computeNameMapping(nameMapping);
        T copy = Delegate.copyMathExpressionSymbol(mathExpressionSymbol);
        NameReplacer.replaceNames(copy, s -> newNameMapping.get(s));
        return copy;
    }

    private static EMAMSymbolicVariableSymbol replaceName(EMAMSymbolicVariableSymbol variableSymbol, Map<String, String> nameMapping) {
        Map<String, String> newNameMapping = computeNameMapping(nameMapping);
        EMAMSymbolicVariableSymbol copy = new EMAMSymbolicVariableSymbol(newNameMapping.get(variableSymbol.getName()));
        copy.setPort(variableSymbol.getPort().get());
        return copy;
    }

    private static Map<String, String> computeNameMapping(Map<String, String> nameMapping) {
        Map<String, String> newNameMapping = new HashMap<>();
        for (Map.Entry<String, String> entry : nameMapping.entrySet()) {
            String newKey = entry.getKey().replace("[", "(").replace("]", ")");
            newNameMapping.put(newKey, entry.getValue());
        }
        return newNameMapping;
    }


    private static EMAMInitialValueSymbol convertInitialValue(EMAPortInstanceSymbol port) {
        EMAMInitialValueSymbol result = new EMAMInitialValueSymbol(port.getNameWithoutArrayBracketPart());
        ASTExpression initialValue = port.getInitialValue();
        if (port.isPartOfPortArray())
            result.setMathMatrixAccessOperatorSymbol(getIndex(port));

        MathExpressionSymbol symbolFromExpression = MathHelper.getSymbolFromExpression(initialValue);
        result.setValue(symbolFromExpression);
        return result;
    }

    private static EMAMInitialGuessSymbol convertInitialGuess(EMAPortInstanceSymbol port) {
        EMAMInitialGuessSymbol result = new EMAMInitialGuessSymbol(port.getNameWithoutArrayBracketPart());
        ASTExpression initialValue = port.getInitialGuess();
        if (port.isPartOfPortArray())
            result.setMathMatrixAccessOperatorSymbol(getIndex(port));

        MathExpressionSymbol symbolFromExpression = MathHelper.getSymbolFromExpression(initialValue);
        result.setValue(symbolFromExpression);
        return result;
    }

    private static MathMatrixAccessOperatorSymbol getIndex(EMAPortInstanceSymbol port) {
        String regex = ".*\\[(\\d+)\\]";
        Pattern pattern = Pattern.compile(regex);
        Matcher matcher = pattern.matcher(port.getName());

        String strIndex = "1";
        while (matcher.find())
            strIndex = matcher.group();


        MathMatrixAccessOperatorSymbol index = new MathMatrixAccessOperatorSymbol();
        MathMatrixAccessSymbol accessSymbol = new MathMatrixAccessSymbol(
                new MathNumberExpressionSymbol(Rational.valueOf(strIndex)));
        index.setMathMatrixAccessSymbols(Collections.singletonList(accessSymbol));

        return index;
    }

    private static class ExpressionConverter {
        private Set<EMAMEquationSymbol> system = new HashSet<>();
        private Set<EMAMSymbolicVariableSymbol> variables = new HashSet<>();

        private void handleExpression(MathAssignmentExpressionSymbol expression) {
            MathAssignmentExpressionSymbol copy = Delegate.copyMathExpressionSymbol(expression);
            String name = copy.getNameOfMathValue();
            incIndexOf(name);
            String currentName = getCurrentNameOf(name);
            NameReplacer replacer = new NameReplacer(s -> getCurrentNameOf(s));
            replacer.handle(copy);

            MathExpressionSymbol leftSide;
            if (expression.getMathMatrixAccessOperatorSymbol() != null)
                leftSide = new MathMatrixNameExpressionSymbol(copy.getNameOfMathValue());
            else
                leftSide = new MathNameExpressionSymbol(copy.getNameOfMathValue());

            EMAMEquationSymbol equationSymbol = new EMAMEquationSymbol();
            equationSymbol.setLeftExpression(leftSide);
            equationSymbol.setRightExpression(copy.getExpressionSymbol());

            if (variables.stream().filter(v -> v.getName().equals(currentName)).collect(Collectors.toSet()).isEmpty()) {
                EMAMSymbolicVariableSymbol emamVariableSymbol = new EMAMSymbolicVariableSymbol(currentName);
                Optional<EMAMSymbolicVariableSymbol> first = variables.stream().filter(v -> v.getName().equals(name)).findFirst();
                if (first.isPresent() && first.get().isPort())
                    emamVariableSymbol.setPort(first.get().getPort().get());
                variables.add(emamVariableSymbol);
            }
            system.add(equationSymbol);
        }

        private Map<String, Integer> lastIndexOf = new HashMap<>();

        private int getLastIndexOf(String var) {
            Integer index = lastIndexOf.get(var);
            if (index == null) return 0;
            return index;
        }

        private void incIndexOf(String var) {
            Integer index = lastIndexOf.get(var);
            if (index == null) index = -1;
            lastIndexOf.put(var, ++index);
        }

        private String getCurrentNameOf(String var) {
            int index = getLastIndexOf(var);
            return index != 0 ? var + "_" + index : var;
        }
    }

}
