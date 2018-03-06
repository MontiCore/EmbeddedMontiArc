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
    private List<CheckCoCoResult> mainPackageModels = new LinkedList<>();

    public void orderTestResults(List<CheckCoCoResult> testResults) {

        Map<String, Map<String, CheckCoCoResult>> maps = mapTestResults(testResults);
        Map<String, Map<String, List<CheckCoCoResult>>> mainPackages = new HashMap<>();

        CheckCoCoResult notParsed   = new CheckCoCoResult("");
        CheckCoCoResult notResolved = new CheckCoCoResult("");
        notParsed.setErrorResult(true);
        notResolved.setErrorResult(true);
        notParsed.setModelName("Parsing failed");
        notResolved.setModelName("Resolving failed");
        notParsed.setProject("Errored_Parsing");
        notResolved.setProject("Errored_Resolving");
        notParsed.setParsed(-1);
        notResolved.setResolved(-1);

        for (CheckCoCoResult testResult : testResults) {
            if (testResult.getParsed() != 1) {
                if(notParsed.getRootFile() == null)
                    notParsed.setRootFile(new File(CheckCoCoResult.erroredString + "_" + testResult.getRootFile().getName()));
                notParsed.addChild(new ChildElement("", testResult));
                testResult.addParent(notParsed);
            } else if (testResult.getResolved() != 1) {
                if(notResolved.getRootFile() == null)
                    notResolved.setRootFile(new File(CheckCoCoResult.erroredString + "_" + testResult.getRootFile().getName()));
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
            if(testResult.getParsed() == 1) {
                String modelName = testResult.getModelName();
                if(modelName.startsWith("."))
                    modelName = modelName.substring(1);
                String mainPackage = modelName.substring(0,modelName.indexOf("."));
                if(mainPackage.equals(""))
                    mainPackage = modelName.substring(1).substring(modelName.substring(1).indexOf("."));

                if(mainPackages.get(testResult.getModelPath()) == null)
                    mainPackages.put(testResult.getModelPath(), new HashMap<>());
                Map<String, List<CheckCoCoResult>> modelPathModels = mainPackages.get(testResult.getModelPath());

                if(modelPathModels.get(mainPackage) == null)
                    modelPathModels.put(mainPackage, new LinkedList<>());
                List<CheckCoCoResult> insideMainPackage = modelPathModels.get(mainPackage);

                insideMainPackage.add(testResult);

            }
        }

        for (CheckCoCoResult testResult : testResults) {
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
            Map<String, List<CheckCoCoResult>> modelPathModels = mainPackages.get(modelPath);

            for(String mainPackage: modelPathModels.keySet()){
                List<CheckCoCoResult> insideMainPackage = modelPathModels.get(mainPackage);

                CheckCoCoResult mainPackageResult = new CheckCoCoResult(modelPath + "." + mainPackage);
                mainPackageModels.add(mainPackageResult);
                mainPackageResult.setMainPackage(true);

                for(CheckCoCoResult testResult: insideMainPackage){
                    if(mainPackageResult.getProject().equals("")) mainPackageResult.setProject(testResult.getProject());
                    mainPackageResult.addChild(new ChildElement(testResult.getModelName(), testResult));
                    if(mainPackageResult.getRootFile() == null) mainPackageResult.setRootFile(testResult.getRootFile());

                    if(testResult.getParsed() != 0 && testResult.getParsed() < mainPackageResult.getParsed() || mainPackageResult.getParsed() == 0) {
                        mainPackageResult.setParsed(testResult.getParsed());
                    }
                    if(testResult.getResolved() != 0 && testResult.getResolved() < mainPackageResult.getResolved() || mainPackageResult.getResolved() == 0) {
                        mainPackageResult.setResolved(testResult.getResolved());
                    }
                    if(testResult.getComponentCapitalized() != 0 && testResult.getComponentCapitalized() < mainPackageResult.getComponentCapitalized() || mainPackageResult.getComponentCapitalized() == 0) {
                        mainPackageResult.setComponentCapitalized(testResult.getComponentCapitalized());
                    }
                    if(testResult.getComponentInstanceNamesUnique() != 0 && testResult.getComponentInstanceNamesUnique() < mainPackageResult.getComponentInstanceNamesUnique() || mainPackageResult.getComponentInstanceNamesUnique() == 0) {
                        mainPackageResult.setComponentInstanceNamesUnique(testResult.getComponentInstanceNamesUnique());
                    }
                    if(testResult.getComponentWithTypeParametersHasInstance() != 0 && testResult.getComponentWithTypeParametersHasInstance() < mainPackageResult.getComponentWithTypeParametersHasInstance() || mainPackageResult.getComponentWithTypeParametersHasInstance() == 0) {
                        mainPackageResult.setComponentWithTypeParametersHasInstance(testResult.getComponentWithTypeParametersHasInstance());
                    }
                    if(testResult.getConnectorEndPointCorrectlyQualified() != 0 && testResult.getConnectorEndPointCorrectlyQualified() < mainPackageResult.getConnectorEndPointCorrectlyQualified() || mainPackageResult.getConnectorEndPointCorrectlyQualified() == 0) {
                        mainPackageResult.setConnectorEndPointCorrectlyQualified(testResult.getConnectorEndPointCorrectlyQualified());
                    }
                    if(testResult.getDefaultParametersHaveCorrectOrder() != 0 && testResult.getDefaultParametersHaveCorrectOrder() < mainPackageResult.getDefaultParametersHaveCorrectOrder() || mainPackageResult.getDefaultParametersHaveCorrectOrder() == 0) {
                        mainPackageResult.setDefaultParametersHaveCorrectOrder(testResult.getDefaultParametersHaveCorrectOrder());
                    }
                    if(testResult.getInPortUniqueSender() != 0 && testResult.getInPortUniqueSender() < mainPackageResult.getInPortUniqueSender() || mainPackageResult.getInPortUniqueSender() == 0) {
                        mainPackageResult.setInPortUniqueSender(testResult.getInPortUniqueSender());
                    }
                    if(testResult.getPackageLowerCase() != 0 && testResult.getPackageLowerCase() < mainPackageResult.getPackageLowerCase() || mainPackageResult.getPackageLowerCase() == 0) {
                        mainPackageResult.setPackageLowerCase(testResult.getPackageLowerCase());
                    }
                    if(testResult.getParameterNamesUnique() != 0 && testResult.getParameterNamesUnique() < mainPackageResult.getParameterNamesUnique() || mainPackageResult.getParameterNamesUnique() == 0) {
                        mainPackageResult.setParameterNamesUnique(testResult.getParameterNamesUnique());
                    }
                    if(testResult.getPortTypeOnlyBooleanOrSIUnit() != 0 && testResult.getPortTypeOnlyBooleanOrSIUnit() < mainPackageResult.getPortTypeOnlyBooleanOrSIUnit() || mainPackageResult.getPortTypeOnlyBooleanOrSIUnit() == 0) {
                        mainPackageResult.setPortTypeOnlyBooleanOrSIUnit(testResult.getPortTypeOnlyBooleanOrSIUnit());
                    }
                    if(testResult.getPortUsage() != 0 && testResult.getPortUsage() < mainPackageResult.getPortUsage() || mainPackageResult.getPortUsage() == 0) {
                        mainPackageResult.setPortUsage(testResult.getPortUsage());
                    }
                    if(testResult.getReferencedSubComponentExists() != 0 && testResult.getReferencedSubComponentExists() < mainPackageResult.getReferencedSubComponentExists() || mainPackageResult.getReferencedSubComponentExists() == 0) {
                        mainPackageResult.setReferencedSubComponentExists(testResult.getReferencedSubComponentExists());
                    }
                    if(testResult.getSimpleConnectorSourceExists() != 0 && testResult.getSimpleConnectorSourceExists() < mainPackageResult.getSimpleConnectorSourceExists() || mainPackageResult.getSimpleConnectorSourceExists() == 0) {
                        mainPackageResult.setSimpleConnectorSourceExists(testResult.getSimpleConnectorSourceExists());
                    }
                    if(testResult.getSourceTargetNumberMatch() != 0 && testResult.getSourceTargetNumberMatch() < mainPackageResult.getSourceTargetNumberMatch() || mainPackageResult.getSourceTargetNumberMatch() == 0) {
                        mainPackageResult.setSourceTargetNumberMatch(testResult.getSourceTargetNumberMatch());
                    }
                    if(testResult.getSubComponentsConnected() != 0 && testResult.getSubComponentsConnected() < mainPackageResult.getSubComponentsConnected() || mainPackageResult.getSubComponentsConnected() == 0) {
                        mainPackageResult.setSubComponentsConnected(testResult.getSubComponentsConnected());
                    }
                    if(testResult.getTopLevelComponentHasNoInstanceName() != 0 && testResult.getTopLevelComponentHasNoInstanceName() < mainPackageResult.getTopLevelComponentHasNoInstanceName() || mainPackageResult.getTopLevelComponentHasNoInstanceName() == 0) {
                        mainPackageResult.setTopLevelComponentHasNoInstanceName(testResult.getTopLevelComponentHasNoInstanceName());
                    }
                    if(testResult.getTypeParameterNamesUnique() != 0 && testResult.getTypeParameterNamesUnique() < mainPackageResult.getTypeParameterNamesUnique() || mainPackageResult.getTypeParameterNamesUnique() == 0) {
                        mainPackageResult.setTypeParameterNamesUnique(testResult.getTypeParameterNamesUnique());
                    }
                    if(testResult.getUniquePorts() != 0 && testResult.getUniquePorts() < mainPackageResult.getUniquePorts() || mainPackageResult.getUniquePorts() == 0) {
                        mainPackageResult.setUniquePorts(testResult.getUniquePorts());
                    }
                    if(testResult.getAtomicComponent() != 0 && testResult.getAtomicComponent() < mainPackageResult.getAtomicComponent() || mainPackageResult.getAtomicComponent() == 0) {
                        mainPackageResult.setAtomicComponent(testResult.getAtomicComponent());
                    }

                    mainPackageResult.setModelName(mainPackageResult.getProject().substring(0, mainPackageResult.getProject().length() - 1)
                            + "." + mainPackage);

                }
            }
        }

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

    public List<CheckCoCoResult> getMainPackageModels() {
        return mainPackageModels;
    }

    public void setMainPackageModels(List<CheckCoCoResult> mainPackageModels) {
        this.mainPackageModels = mainPackageModels;
    }
}
