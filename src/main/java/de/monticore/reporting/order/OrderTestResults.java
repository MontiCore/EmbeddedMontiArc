package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbolReference;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class OrderTestResults {

    public static List<CheckCoCoResult> orderTestResults(List<CheckCoCoResult> testResults) {

        Map<String, Map<String, CheckCoCoResult>> maps = mapTestResults(testResults);

        for(CheckCoCoResult testResult: testResults) {
            if(testResult.getResolved() != 1) continue;

            ASTComponent ast = (ASTComponent) testResult.getResolvedAst();
            String modelPath = testResult.getModelPath();
            Map<String, CheckCoCoResult> modelPathMap = maps.get(modelPath);

            for(ASTElement element: ast.getBody().getElements()){
                if ( element instanceof ASTSubComponent ){
                    if(((ASTSubComponent) element).getInstances().size() > 0) {
                        ComponentInstanceSymbol instanceSymbol = (ComponentInstanceSymbol) element.getSymbol().get();
                        ComponentSymbolReference symbolReference = instanceSymbol.getComponentType();
                        ComponentSymbol commonSymbolReference = symbolReference.getReferencedSymbol();

                        String name = commonSymbolReference.getFullName();
                        CheckCoCoResult child = modelPathMap.get(name);

                        String referencedName = ((ASTSubComponent) element).getInstances().get(0).getName();
                        ChildElement childElement = new ChildElement(referencedName, child);
                        testResult.addChild(childElement);
                        if (child != null)
                            child.addParent(testResult);
                    }
                }
            }
        }

        List<CheckCoCoResult> rootModels = new LinkedList<>();
        List<CheckCoCoResult> parentModels = new LinkedList<>();
        List<CheckCoCoResult> moreThanOneParent = new LinkedList<>();

        for(CheckCoCoResult testResult: testResults){
            if(testResult.getResolved() != 1) continue;
            if(testResult.getChildren().size() > 0)
                parentModels.add(testResult);
            else if (testResult.getParents().size() > 1)
                moreThanOneParent.add(testResult);
            if(testResult.getParents().size() == 0 && testResult.getChildren().size() > 0)
                rootModels.add(testResult);
        }

        return rootModels;
    }

    private static Map<String, Map<String, CheckCoCoResult>> mapTestResults(List<CheckCoCoResult> testResults){
        Map<String, Map<String, CheckCoCoResult>> res = new HashMap<>();

        for(CheckCoCoResult testResult: testResults) {
            if(testResult.getParsed() != 1) continue;
            String modelPath = testResult.getModelPath();
            String modelName = testResult.getModelName();
            if(res.get(modelPath) == null) {
                res.put(modelPath, new HashMap<>());
            }
            res.get(modelPath).put(modelName, testResult);
        }

        return res;
    }


}
