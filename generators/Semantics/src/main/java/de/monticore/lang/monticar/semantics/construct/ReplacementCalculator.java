/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.construct;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.monticar.semantics.Options;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.resolve.ConstantsCalculator;
import de.monticore.lang.monticar.semantics.resolve.SymbolTableHelper;
import de.monticore.lang.monticar.semantics.util.math.MathHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;

import java.io.IOException;
import java.io.StringReader;
import java.util.*;
import java.util.stream.Collectors;

public class ReplacementCalculator {

    public static void generateAndReplaceComponent(TaggingResolver globalScope, EMAComponentInstanceSymbol symbol,
                                                   Map<EMAPortInstanceSymbol, String> solutions) {

        MathComponentGenerator generator = new MathComponentGenerator();

        String parentComponentName = symbol.getParent().isPresent() ? symbol.getParent().get().getFullName() : symbol.getPackageName();
        String packageName = Options.synthPackagePreFix + "." + parentComponentName;
        Scope enclosingScope = symbol.getEnclosingScope();
        String type = StringUtils.capitalize(symbol.getName());
        Map<String, String> inports = new HashMap<>();
        Map<String, String> outports = new HashMap<>();
        List<String> mathStatements = new LinkedList<>();

        for (EMAPortInstanceSymbol inport : symbol.getIncomingPortInstances()) {
            inports.put(inport.getName(), inport.getTypeReference().getName());
        }
        for (EMAPortInstanceSymbol outport : symbol.getOutgoingPortInstances()) {
            outports.put(outport.getName(), outport.getTypeReference().getName());
            mathStatements.add(outport.getName() + "=" + solutions.get(outport));
        }

        // Analyze math Statements in order to redirect new input ports
        // Dependend constants should be output ports of some components or the input ports of the rootcomponent
        Set<String> dependendConstants = getDependedConstants(mathStatements, outports.keySet());
        Set<EMAPortInstanceSymbol> dependingPorts = new HashSet<>();
        for (String dependendConstant : dependendConstants) {
            if (!isTime(dependendConstant)) {
                Optional<EMAPortInstanceSymbol> port = symbol.getSpannedScope().resolve(dependendConstant, EMAPortInstanceSymbol.KIND);
                if (!port.isPresent()) Log.error("TODO cannot find dependend port, is probably a non resolved parameter");
                dependingPorts.add(port.get());
            }
        }

        for (EMAPortInstanceSymbol dependingPort : dependingPorts) {
            String nameOfInputPort = "";
            Optional<EMAPortInstanceSymbol> inport = getAlreadyConnectingPort(symbol, dependingPort);
            if (inport.isPresent()) {
                nameOfInputPort = inport.get().getName();
            } else {
                // New port
                nameOfInputPort = NameHelper.replaceWithUnderScore(dependingPort.getFullName());
                String portType = dependingPort.getTypeReference().getReferencedSymbol().getName();
                inports.put(nameOfInputPort, portType);
                EMADynamicPortInstanceSymbol newPort = InstanceCreator.createPortInstanceSymbol(
                        nameOfInputPort,
                        symbol.getName(),
                        portType,
                        true,
                        symbol.getSpannedScope().getAsMutableScope());

                // Calculate redirections
                ConnectSourceWithTargetPort.addConnection(dependingPort, newPort);
            }

            // Replace the name of the depening port with the existing connecting / new connecting port
            ListIterator<String> statement = mathStatements.listIterator();
            while (statement.hasNext()) {
                String s = statement.next();
                statement.set(s.replace(dependingPort.getFullName(), nameOfInputPort));
            }
        }

        generator.generate(type, packageName, inports, outports, mathStatements, Options.synthPath);

        // Remove old instance
        SymbolTableHelper.removeComponent(symbol);

        // Resolve new instance
        String fullQualifiedName = NameHelper.toInstanceFullQualifiedName(packageName,
                type);
        SymbolTableHelper.resolveInstanceTo(globalScope, fullQualifiedName, enclosingScope, parentComponentName);

    }

    private static boolean isTime(String s) {
        return Options.timeName.equals(s);
    }

    private static Optional<EMAPortInstanceSymbol> getAlreadyConnectingPort(EMAComponentInstanceSymbol component,
                                                                     EMAPortInstanceSymbol port) {
        Collection<EMAPortInstanceSymbol> atomicTargetsOf = SymbolTableHelper.getAtomicTargetsOf(port);
        for (EMAPortInstanceSymbol targetPort : atomicTargetsOf)
            if (targetPort.getComponentInstance().equals(component))
                return Optional.of(targetPort);
        return Optional.empty();
    }

    private static Set<String> getDependedConstants(List<String> mathStatements, Set<String> variables) {
        Set<String> res = new HashSet<>();
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        for (String mathStatement : mathStatements) {
            ASTExpression expr = null;
            Map<String, String> backMapping = new HashMap<>();
            mathStatement = MathHelper.replaceQualifiedNamesWithUnderscores(mathStatement, backMapping);
            try {
                expr = parser.parseExpression(new StringReader(mathStatement)).get();
            } catch (IOException e) {
                e.printStackTrace();
            }

            ConstantsCalculator constantsCalculator = new ConstantsCalculator(variables);
            expr.accept(constantsCalculator);
            res.addAll(constantsCalculator.getConstants().stream().map(
                    s -> backMapping.getOrDefault(s, s)).collect(Collectors.toSet()));
        }
        return res;
    }
}
