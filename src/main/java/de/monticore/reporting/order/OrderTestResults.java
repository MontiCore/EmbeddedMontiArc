package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbolReference;
import de.monticore.reporting.testCocos.helper.CheckCoCoResult;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class OrderTestResults {


    private List<CheckCoCoResult> rootModels = new LinkedList<>();
    private List<CheckCoCoResult> hasNoParentModels = new LinkedList<>();


    public void orderTestResults(List<CheckCoCoResult> testResults) {

        Map<String, Map<String, CheckCoCoResult>> maps = mapTestResults(testResults);

        CheckCoCoResult notParsed   = new CheckCoCoResult("");
        CheckCoCoResult notResolved = new CheckCoCoResult("");
        notParsed.setModelName("Parsing failed");
        notResolved.setModelName("Resolving failed");
        notParsed.setRootFile(new File(CheckCoCoResult.erroredString));
        notResolved.setRootFile(new File(CheckCoCoResult.erroredString));
        notParsed.setProject("Errored_Parsing");
        notResolved.setProject("Errored_Resolving");
        notParsed.setParsed(-1);
        notResolved.setResolved(-1);

        for (CheckCoCoResult testResult : testResults) {
            if (testResult.getParsed() != 1) {
                notParsed.addChild(new ChildElement("", testResult));
                testResult.addParent(notParsed);
            } else if (testResult.getResolved() != 1) {
                notResolved.addChild(new ChildElement("", testResult));
                testResult.addParent(notResolved);
            } else {

                ASTComponent ast = (ASTComponent) testResult.getResolvedAst();
                String modelPath = testResult.getModelPath();
                Map<String, CheckCoCoResult> modelPathMap = maps.get(modelPath);

                for (ASTElement element : ast.getBody().getElements()) {
                    if (element instanceof ASTSubComponent) {
                        if (((ASTSubComponent) element).getInstances().size() > 0) {
                            ComponentInstanceSymbol instanceSymbol = (ComponentInstanceSymbol) element.getSymbol().get();
                            ComponentSymbolReference symbolReference = instanceSymbol.getComponentType();
                            ComponentSymbol commonSymbolReference = symbolReference.getReferencedSymbol();

                            String name = commonSymbolReference.getFullName();
                            CheckCoCoResult child = modelPathMap.get(name);

                            if(child != null) {

                                String referencedName = ((ASTSubComponent) element).getInstances().get(0).getName();
                                ChildElement childElement = new ChildElement(referencedName, child);
                                testResult.addChild(childElement);
                                child.addParent(testResult);
                            }
                        }
                    }
                }
            }
        }

        for (CheckCoCoResult testResult : testResults) {
            if (testResult.getResolved() != 1) continue;
            if (testResult.getParents().size() == 0 && testResult != notParsed && testResult != notResolved)
                getHasNoParentModels().add(testResult);
            if (testResult.getParents().size() == 0 && testResult.getChildren().size() > 0 && testResult != notParsed && testResult != notResolved)
                getRootModels().add(testResult);
        }

        if(notParsed.getChildren().size() > 0)
            hasNoParentModels.add(notParsed);
        if(notResolved.getChildren().size() > 0)
            hasNoParentModels.add(notResolved);

    }

    private Map<String, Map<String, CheckCoCoResult>> mapTestResults(List<CheckCoCoResult> testResults) {
        Map<String, Map<String, CheckCoCoResult>> res = new HashMap<>();

        for (CheckCoCoResult testResult : testResults) {
            if (testResult.getParsed() != 1) continue;
            String modelPath = testResult.getModelPath();
            String modelName = testResult.getModelName();
            if (res.get(modelPath) == null) {
                res.put(modelPath, new HashMap<>());
            }
            res.get(modelPath).put(modelName, testResult);
        }

        return res;
    }


    public List<CheckCoCoResult> getRootModels() {
        return rootModels;
    }

    public void setRootModels(List<CheckCoCoResult> rootModels) {
        this.rootModels = rootModels;
    }

    public List<CheckCoCoResult> getHasNoParentModels() {
        return hasNoParentModels;
    }

    public void setHasNoParentModels(List<CheckCoCoResult> hasNoParentModels) {
        this.hasNoParentModels = hasNoParentModels;
    }
}
