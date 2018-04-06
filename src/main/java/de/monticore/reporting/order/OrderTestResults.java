package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbolReference;
import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.helper.OrderableModelInfoCreator;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class OrderTestResults <T extends OrderableModelInfo>{


    private List<T> rootModels = new LinkedList<>();
    private List<T> hasNoParentModels = new LinkedList<>();
    private List<T> mainPackageModels = new LinkedList<>();

    public void orderTestResults(List<T> testResults, OrderableModelInfoCreator<T> creator) {

        Map<String, Map<String, T>> maps = mapTestResults(testResults);
        Map<String, Map<String, List<T>>> mainPackages = new HashMap<>();

        T notParsed   = creator.createNewInstance("");
        T notResolved = creator.createNewInstance("");
        notParsed.setErrorResult(true);
        notResolved.setErrorResult(true);
        notParsed.setModelName("Parsing failed");
        notResolved.setModelName("Resolving failed");
        notParsed.setProject("Errored_Parsing");
        notResolved.setProject("Errored_Resolving");
        notParsed.setParsed(-1);
        notResolved.setResolved(-1);

        for (T testResult : testResults) {
            if (testResult.getParsed() != 1) {
                if(notParsed.getRootName1().equals(""))
                    notParsed.setRootName1(OrderableModelInfo.erroredString + "_" + testResult.getRootName1());
                testResult.setRootName1(notParsed.getRootName1());
                notParsed.addChild(new ChildElement("", testResult));
                testResult.addParent(notParsed);
            } else if (testResult.getResolved() != 1) {
                if(notResolved.getRootName1().equals(""))
                    notResolved.setRootName1(OrderableModelInfo.erroredString + "_" + testResult.getRootName1());
                notResolved.addChild(new ChildElement("", testResult));
                testResult.addParent(notResolved);
            } else {

                ASTComponent ast = (ASTComponent) testResult.getResolvedAST();
                String modelPath = testResult.getModelPath();
                Map<String, T> modelPathMap = maps.get(modelPath);

                for (ASTElement element : ast.getBody().getElements()) {
                    if (element instanceof ASTSubComponent) {
                        if (((ASTSubComponent) element).getInstances().size() > 0) {
                            ComponentInstanceSymbol instanceSymbol = (ComponentInstanceSymbol) element.getSymbol().get();
                            ComponentSymbolReference symbolReference = instanceSymbol.getComponentType();
                            ComponentSymbol commonSymbolReference = symbolReference.getReferencedSymbol();

                            String name = commonSymbolReference.getFullName();
                            OrderableModelInfo child = modelPathMap.get(name);

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
            if(testResult.getParsed() == 1) {
                String modelName = testResult.getModelName();
                if(modelName.startsWith("."))
                    modelName = modelName.substring(1);
                String mainPackage = null;
                try {
                    mainPackage = modelName.substring(0,modelName.indexOf("."));
                } catch (Exception e) {
                    e.printStackTrace();
                }

                if(mainPackage == null){
                    mainPackage = "FailedToFindMainPackage";
                }
                else if(mainPackage.equals(""))
                    mainPackage = modelName.substring(1).substring(modelName.substring(1).indexOf("."));

                if(mainPackages.get(testResult.getModelPath()) == null)
                    mainPackages.put(testResult.getModelPath(), new HashMap<>());
                Map<String, List<T>> modelPathModels = mainPackages.get(testResult.getModelPath());

                if(modelPathModels.get(mainPackage) == null)
                    modelPathModels.put(mainPackage, new LinkedList<>());
                List<T> insideMainPackage = modelPathModels.get(mainPackage);

                insideMainPackage.add(testResult);

            }
        }

        for (T testResult : testResults) {
            if (testResult.getResolved() != 1) continue;
            if (testResult.getParents().size() == 0 && testResult != notParsed && testResult != notResolved)
                getHasNoParentModels().add(testResult);
            if (testResult.getParents().size() == 0 && testResult.getChildren().size() > 0 && testResult != notParsed && testResult != notResolved)
                getRootModels().add(testResult);
        }

        if(notParsed.getChildren().size() > 0) {
            hasNoParentModels.add(notParsed);
            mainPackageModels.add(notParsed);
        }
        if(notResolved.getChildren().size() > 0) {
            hasNoParentModels.add(notResolved);
            mainPackageModels.add(notResolved);
        }

        for(String modelPath: mainPackages.keySet()){
            Map<String, List<T>> modelPathModels = mainPackages.get(modelPath);

            for(String mainPackage: modelPathModels.keySet()){
                List<T> insideMainPackage = modelPathModels.get(mainPackage);

                T mainPackageResult = creator.createNewInstance(modelPath + "." + mainPackage);
                mainPackageModels.add(mainPackageResult);
                mainPackageResult.setMainPackage(true);

                for(OrderableModelInfo testResult: insideMainPackage){
                    if(mainPackageResult.getProject().equals("")) mainPackageResult.setProject(testResult.getProject());
                    mainPackageResult.addChild(new ChildElement(testResult.getModelName(), testResult));
                    if(mainPackageResult.getRootFile1() == null) mainPackageResult.setRootFile1(testResult.getRootFile1());
                    if(mainPackageResult.getRootName1().equals("")) mainPackageResult.setRootName1(testResult.getRootName1());
                    if(mainPackageResult.getModelPath().equals("")) mainPackageResult.setModelPath(testResult.getModelPath());

                    if(mainPackageResult.getFileType().equals("")) mainPackageResult.setFileType(testResult.getFileType());
                    else if(!mainPackageResult.getFileType().equals("EMA/EMAM") && !mainPackageResult.getFileType().equals(testResult.getFileType()))
                        mainPackageResult.setFileType("EMA/EMAM");

                    if(testResult.getParsed() != 0 && testResult.getParsed() < mainPackageResult.getParsed() || mainPackageResult.getParsed() == 0) {
                        mainPackageResult.setParsed(testResult.getParsed());
                    }
                    if(testResult.getResolved() != 0 && testResult.getResolved() < mainPackageResult.getResolved() || mainPackageResult.getResolved() == 0) {
                        mainPackageResult.setResolved(testResult.getResolved());
                    }
                    mainPackageResult.setChildInfo();

                    mainPackageResult.setModelName(mainPackageResult.getProject().substring(0, mainPackageResult.getProject().length() - 1)
                            + "." + mainPackage);

                }
            }
        }

    }

    private Map<String, Map<String, T>> mapTestResults(List<T> testResults) {
        Map<String, Map<String, T>> res = new HashMap<>();

        for (T testResult : testResults) {
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


    public List<T> getRootModels() {
        return rootModels;
    }


    public List<T> getHasNoParentModels() {
        return hasNoParentModels;
    }


    public List<T> getMainPackageModels() {
        return mainPackageModels;
    }
}
