/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.resolve;

import de.monticore.assignmentexpressions._ast.ASTRegularAssignmentExpression;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.EmbeddedMontiArcMathMill;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.math._ast.ASTMathAssignmentStatement;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.monticar.semantics.construct.ComponentReplacement;
import de.monticore.lang.monticar.semantics.construct.MathComponentGenerator;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopAnalyzer;
import de.monticore.lang.monticar.semantics.loops.analyze.LoopKind;
import de.monticore.lang.monticar.semantics.loops.detection.Detection;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.lang.monticar.semantics.loops.graph.EMAEdge;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAPort;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.lang.monticar.semantics.solver.linearsolver.LinearEquationSystemSolver;
import de.monticore.lang.monticar.semantics.solver.linearsolver.MathEclipseSolver;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.io.StringReader;
import java.util.*;
import java.util.stream.Collectors;

public class Resolver {
    private boolean solveNumeric;
    private String rootModel;

    public Resolver(String rootModel) {
        this.rootModel = rootModel;
    }

    public TaggingResolver createSymTab(GlobalScope scope) {
        EMAComponentSymbol component = scope.<EMAComponentSymbol>resolve(rootModel, EMAComponentSymbol.KIND).orElse(null);
//        fix(component);

        Detection detection = new Detection();
        Set<StrongConnectedComponent> strongConnectedComponents = detection.detectLoops(component);
        Set<ComponentReplacement> replacements = new HashSet<>();

        for (StrongConnectedComponent strongConnectedComponent : strongConnectedComponents) {
            LoopAnalyzer analyzer = new LoopAnalyzer();
            analyzer.analyze(strongConnectedComponent);
            if (strongConnectedComponent.getKind().equals(LoopKind.QuadraticLinear)) {
                replacements = handleLinear(strongConnectedComponent);
            } else {
                Log.error("0x907651 not yet supported");
            }
        }

        GlobalScope symTab = SymtabCreator.createSymTabForReplacement(replacements, "src/test/resources", "src/main/resources", "target/generated-components");
        TaggingResolver tagging = new TaggingResolver(symTab, Arrays.asList("src/test/resources", "src/main/resources", "target/generated-components"));
//        TagMinMaxTagSchema.registerTagTypes(tagging);
//        TagTableTagSchema.registerTagTypes(tagging);
//        TagBreakpointsTagSchema.registerTagTypes(tagging);
//        TagExecutionOrderTagSchema.registerTagTypes(tagging);
//        TagInitTagSchema.registerTagTypes(tagging);
//        TagThresholdTagSchema.registerTagTypes(tagging);
//        TagDelayTagSchema.registerTagTypes(tagging);
        return tagging;
    }

    private Set<ComponentReplacement> handleLinear(StrongConnectedComponent strongConnectedComponent) {
        MathEclipseSolver msolver = new MathEclipseSolver();
        LinearEquationSystemSolver solver = new LinearEquationSystemSolver(msolver, msolver);

        Set<ASTExpression> system = new HashSet<>();
        Set<String> variables = new HashSet<>();

        for (EMAPort emaPort : strongConnectedComponent.getPortStatements().keySet()) {
            variables.add(emaPort.getFullName());
            ASTMathAssignmentStatement statement = strongConnectedComponent.getPortStatements().get(emaPort);
            ASTNameExpression nameExpression = EmbeddedMontiArcMathMill.nameExpressionBuilder()
                    .setName(statement.getName()).build();
            ASTRegularAssignmentExpression expression = EmbeddedMontiArcMathMill.regularAssignmentExpressionBuilder()
                    .setLeftExpression(nameExpression)
                    .setRightExpression(statement.getExpression())
                    .setOperator(statement.getMathAssignmentOperator().getOperator())
                    .build();
            system.add(expression);
        }

        Map<String, String> solutions = solver.solveLinearEquationSystem(system, variables);

        Set<EMAVertex> componentsToReplace = calculateComponentsToBreakLoops(strongConnectedComponent);

        return calculateReplacementsAndGenerateComponents(strongConnectedComponent, componentsToReplace, solutions);
    }

    private Set<ComponentReplacement> calculateReplacementsAndGenerateComponents
            (StrongConnectedComponent strongConnectedComponent, Set<EMAVertex> componentsToReplace, Map<String, String> solutions) {

        Set<ComponentReplacement> replacements = new HashSet<>();
        MathComponentGenerator generator = new MathComponentGenerator();

        for (EMAVertex emaVertex : componentsToReplace) {
            String fullName = emaVertex.getFullName();
            String parentComponent = getParentComponent(fullName);
            String type = emaVertex.getReferencedSymbol().getComponentType().getName() + "_synth";
            String packageName = emaVertex.getReferencedSymbol().getPackageName() + ".synth";
            String path = "target/generated-components";
            Map<String, String> inports = new HashMap<>();
            Map<String, String> outports = new HashMap<>();
            List<String> mathStatements = new LinkedList<>();

            for (EMAPort inport : emaVertex.getInports()) {
                inports.put(inport.getName(), inport.getReferencedPort().getTypeReference().getName());
            }
            for (EMAPort outport : emaVertex.getOutports()) {
                outports.put(outport.getName(), outport.getReferencedPort().getTypeReference().getName());
                mathStatements.add(outport.getName() + "=" + solutions.get(outport.getFullName()));
            }

            // Analyze math Statements in order to redirect new input ports
            Set<String> dependendConstants = getDependedConstants(mathStatements, outports.keySet());
            for (String dependendConstant : dependendConstants) {
                Optional<EMAPort> inport = getAlreadyConnectingPort(strongConnectedComponent, emaVertex, dependendConstant);
                if (inport.isPresent()) {
                    ListIterator<String> statement = mathStatements.listIterator();
                    while(statement.hasNext()) {
                        String s = statement.next();
                        statement.set(s.replace(dependendConstant, inport.get().getName()));
                    }
                } else {
                    // TODO
                }
            }




            generator.generate(type, packageName, inports, outports, mathStatements, path);

            ComponentReplacement replacement = new ComponentReplacement(parentComponent, emaVertex.getName(),
                    packageName, type, emaVertex.getName());
            replacements.add(replacement);
        }
        return replacements;

    }

    private Optional<EMAPort> getAlreadyConnectingPort(StrongConnectedComponent strongConnectedComponent, EMAVertex emaVertex, String port) {
        EMAGraph graph = strongConnectedComponent.getGraph();
        List<EMAEdge> edgesWithSourcePort = graph.getEdgesWithSourcePort(graph.getPortMap().get(port));
        for (EMAEdge emaEdge : edgesWithSourcePort) {
            if (emaEdge.getTargetVertex() == emaVertex) {
                return Optional.of(emaEdge.getTargetPort());
            }
        }
        return Optional.empty();
    }

    private String getParentComponent(String fullName) {
        if (fullName.contains("."))
            return fullName.substring(0, fullName.lastIndexOf("."));
        else
            return fullName;
    }

    private Set<String> getDependedConstants(List<String> mathStatements, Set<String> variables) {
        Set<String> res = new HashSet<>();
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        for (String mathStatement : mathStatements) {
            ASTExpression expr = null;
            try {
                expr = parser.parseExpression(new StringReader(mathStatement)).get();
            } catch (IOException e) {
                e.printStackTrace();
            }

            ConstantsCalculator constantsCalculator = new ConstantsCalculator(variables);
            expr.accept(constantsCalculator);
            res.addAll(constantsCalculator.getConstants());
        }
        return res;
    }

    private Set<EMAVertex> calculateComponentsToBreakLoops(StrongConnectedComponent strongConnectedComponent) {
        Set<EMAVertex> res = new HashSet<>();
        for (SimpleCycle simpleCycle : strongConnectedComponent.getSimpleCycles()) {
            Set<EMAVertex> outs = simpleCycle.getOutports().stream().map(o -> o.getEmaVertex()).collect(Collectors.toSet());
            res.addAll(outs);
        }
        return res;
    }


//    private void fix(EMAComponentSymbol component) {
//        for (EMAComponentInstantiationSymbol subComponent : component.getSubComponents()) {
//            fix(subComponent);
//        }
//    }
//
//    private void fix(EMAComponentInstantiationSymbol componentSymbol) {
//        addDefineGenerics(componentSymbol);
//        for (EMAComponentInstantiationSymbol subComponent : componentSymbol.getComponentType().getSubComponents()) {
//            fix(subComponent);
//        }
//    }
//
//    public void addDefineGenerics(EMAComponentInstantiationSymbol componentSymbol) {
//        if (componentSymbol.getInstanceInformation().isPresent()) {
//            int index = 0;
//            Log.info(componentSymbol.getName(), "HasInstanceInformation:");
//            for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : componentSymbol.getComponentType().getResolutionDeclarationSymbols()) {
//                Log.info(resolutionDeclarationSymbol.getNameToResolve(), "ResDecl:");
//                ASTSubComponent subComponent = componentSymbol.getInstanceInformation().get().getASTSubComponent();
//                int number = InstanceInformation.getInstanceNumberFromASTSubComponent(subComponent, index);
//                //if(resolutionDeclarationSymbol.getNameToResolve().equals("targetEigenvectors")){
//                Log.info(subComponent.toString(), "InfoKK:");
//                //}
//                if (number == -1) {
//                    // try with ast
//                    if (resolutionDeclarationSymbol.getASTResolution() instanceof ASTUnitNumberResolution) {
//                        number = ((ASTUnitNumberResolution) resolutionDeclarationSymbol.getASTResolution()).getNumber().get().intValue();
//                    } else {
//                        Log.info(subComponent.toString(), "No number added for" + resolutionDeclarationSymbol.getNameToResolve());
//                        ++index;
//                        break;
//                    }
//                }
//                fixSubComponentInstanceNumbers(componentSymbol, resolutionDeclarationSymbol.getNameToResolve(), number, index);
//                ++index;
//
//            }
//        }
//    }
//
//    public void fixSubComponentInstanceNumbers(EMAComponentInstantiationSymbol componentInstanceSymbol, String nameToResolve, int numberToSet, int index) {
//        for (EMAComponentInstantiationSymbol instanceSymbol : componentInstanceSymbol.getComponentType().getSubComponents()) {
//
//            for (ResolutionDeclarationSymbol resolutionDeclarationSymbol : instanceSymbol.getComponentType().getResolutionDeclarationSymbols()) {
//                Log.info(resolutionDeclarationSymbol.getNameToResolve(), "ResDeclFix:");
//                ASTSubComponent subComponent = instanceSymbol.getInstanceInformation().get().getASTSubComponent();
//                int number = InstanceInformation.getInstanceNumberFromASTSubComponent(subComponent, index);
//                if (number == -1) {
//                    //if(resolutionDeclarationSymbol.getNameToResolve().equals(nameToResolve))
//                    {
//                        Log.info(resolutionDeclarationSymbol.getNameToResolve()+"set number to "+numberToSet+"; oldValue: "+number, "Fixed");
//                        InstanceInformation.setInstanceNumberInASTSubComponent(subComponent, nameToResolve, numberToSet);
//                        break;
//                    }/*else{
//                        Log.info("realName:"+resolutionDeclarationSymbol.getNameToResolve() +" nameToResolve:"+nameToResolve,"Not fixing:");
//                    }*/
//                }
//                ++index;
//            }
//        }
//    }
}
