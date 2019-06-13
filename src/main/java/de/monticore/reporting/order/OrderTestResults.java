package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbolReference;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.helper.OrderableModelInfoCreator;
import de.monticore.symboltable.ImportStatement;
import de.monticore.types.types._ast.ASTSimpleReferenceType;

import java.io.File;
import java.util.*;
import java.util.regex.Pattern;

public class OrderTestResults<T extends OrderableModelInfo> {


    private List<T> rootModels = new LinkedList<>();
    private List<T> hasNoParentModels = new LinkedList<>();
    private List<T> mainPackageModels = new LinkedList<>();

    public void orderTestResults(List<T> testResults, OrderableModelInfoCreator<T> creator) {

        Map<String, Map<String, T>> maps = mapTestResults(testResults);
        Map<String, Map<String, List<T>>> mainPackages = new HashMap<>();

        T notParsed = creator.createNewInstance("");
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
                if (notParsed.getRootName1().equals(""))
                    notParsed.setRootName1(OrderableModelInfo.erroredString + "_" + testResult.getRootName1());
                testResult.setRootName1(notParsed.getRootName1());
                notParsed.addChild(new ChildElement("", testResult));
                testResult.addParent(notParsed);
            } else if (testResult.getResolved() != 1) {
                if (notResolved.getRootName1().equals(""))
                    notResolved.setRootName1(OrderableModelInfo.erroredString + "_" + testResult.getRootName1());
                testResult.setRootName1(notResolved.getRootName1());
                notResolved.addChild(new ChildElement("", testResult));
                testResult.addParent(notResolved);
            } else {

                EMAComponentSymbol ast = testResult.getResolvedAST();
                String modelPath = testResult.getModelPath();
                Map<String, T> modelPathMap = maps.get(modelPath);

                for (ASTSubComponent element : ((ASTComponent) ast.getAstNode().get()).getSubComponents()) {

                    testResult.setAtomic(false);

                    String type = ((ASTSimpleReferenceType) element.getType()).getName(0);
                    String name = "";
                    for (ImportStatement importStatement: ast.getImports()) {
                        if (importStatement.getStatement().contains(".") &&
                                importStatement.getStatement().substring(importStatement
                                .getStatement().lastIndexOf(".")).equals(type)
                                || importStatement.getStatement().equals(type)) {
                            name = importStatement.getStatement();
                            break;
                        }
                    }
                    if (!name.contains(ast.getPackageName()))
                        name = ast.getPackageName() + "." + name;
                    OrderableModelInfo child = modelPathMap.get(name);

                    if (child != null) {

                        String referencedName = element.getInstancesList().get(0).getName();
                        ChildElement childElement = new ChildElement(referencedName, child);
                        testResult.addChild(childElement);
                        child.addParent(testResult);
                    }

                }
            }
            if (testResult.getParsed() == 1) {
                String modelName = testResult.getModelName();
                if (modelName.startsWith("."))
                    modelName = modelName.substring(1);
                String mainPackage = null;
                try {
                    mainPackage = modelName.substring(0, modelName.indexOf("."));
                } catch (Exception e) {
                    e.printStackTrace();
                }

                if (mainPackage == null) {
                    mainPackage = "No Package";
                } else if (mainPackage.equals(""))
                    mainPackage = modelName.substring(1).substring(modelName.substring(1).indexOf("."));

                if (mainPackages.get(testResult.getModelPath()) == null)
                    mainPackages.put(testResult.getModelPath(), new HashMap<>());
                Map<String, List<T>> modelPathModels = mainPackages.get(testResult.getModelPath());

                if (modelPathModels.get(mainPackage) == null)
                    modelPathModels.put(mainPackage, new LinkedList<>());
                List<T> insideMainPackage = modelPathModels.get(mainPackage);

                insideMainPackage.add(testResult);

            }
        }

        for (T testResult : testResults) {
            if (testResult.getResolved() != 1) continue;
            if (testResult.getParents().size() == 0 && testResult != notParsed && testResult != notResolved)
                getHasNoParentModels().add(testResult);
            if (testResult.getParents().size() == 0 && (testResult.getChildren().size() > 0 || !testResult.isAtomic())
                    && testResult != notParsed && testResult != notResolved)
                getRootModels().add(testResult);
        }

        if (notParsed.getChildren().size() > 0) {
            hasNoParentModels.add(notParsed);
            mainPackageModels.add(notParsed);
        }
        if (notResolved.getChildren().size() > 0) {
            hasNoParentModels.add(notResolved);
            mainPackageModels.add(notResolved);
        }

        for (String modelPath : mainPackages.keySet()) {
            Map<String, List<T>> modelPathModels = mainPackages.get(modelPath);

            for (String mainPackage : modelPathModels.keySet()) {
                List<T> insideMainPackage = modelPathModels.get(mainPackage);

                T mainPackageResult = creator.createNewInstance(modelPath + "." + mainPackage);
                mainPackageModels.add(mainPackageResult);
                mainPackageResult.setMainPackage(true);

                for (OrderableModelInfo testResult : insideMainPackage) {
                    if (mainPackageResult.getProject().equals(""))
                        mainPackageResult.setProject(testResult.getProject());
                    mainPackageResult.addChild(new ChildElement(testResult.getModelName(), testResult));
                    if (mainPackageResult.getRootFile1() == null)
                        mainPackageResult.setRootFile1(testResult.getRootFile1());
                    if (mainPackageResult.getRootName1().equals(""))
                        mainPackageResult.setRootName1(testResult.getRootName1());
                    if (mainPackageResult.getModelPath().equals(""))
                        mainPackageResult.setModelPath(testResult.getModelPath());

                    if (mainPackageResult.getFileType().equals(""))
                        mainPackageResult.setFileType(testResult.getFileType());
                    else if (!mainPackageResult.getFileType().equals("EMA/EMAM") && !mainPackageResult.getFileType().equals(testResult.getFileType()))
                        mainPackageResult.setFileType("EMA/EMAM");

                    if (testResult.getParsed() != 0 && testResult.getParsed() < mainPackageResult.getParsed() || mainPackageResult.getParsed() == 0) {
                        mainPackageResult.setParsed(testResult.getParsed());
                    }
                    if (testResult.getResolved() != 0 && testResult.getResolved() < mainPackageResult.getResolved() || mainPackageResult.getResolved() == 0) {
                        mainPackageResult.setResolved(testResult.getResolved());
                    }
                    mainPackageResult.setChildInfo();

                    mainPackageResult.setModelName(mainPackageResult.getProject().substring(0, mainPackageResult.getProject().length() - 1)
                            + "." + mainPackage);

                }
            }
        }
        makeMainModelNamesUnique();
        fixMainModelNames();
    }

    private void fixMainModelNames() {
        for (OrderableModelInfo mainModel: this.mainPackageModels)
            mainModel.setModelName(mainModel.getModelName().replaceFirst(Pattern.quote("."), " - "));
    }

    private void makeMainModelNamesUnique() {
        boolean allUnique = true;
        Map<String, List<OrderableModelInfo>> mainPackagesMap = new HashMap<>();
        for (OrderableModelInfo mainPackage: this.mainPackageModels) {
            String name = mainPackage.getModelName();
            if (mainPackagesMap.get(name) == null)
                mainPackagesMap.put(name, new LinkedList<>());
            mainPackagesMap.get(name).add(mainPackage);
        }

        for (String mainPackageName : mainPackagesMap.keySet()){
            List<OrderableModelInfo> mainPackageWithEqualNames = mainPackagesMap.get(mainPackageName);
            if (mainPackageWithEqualNames.size() > 1){
                allUnique = false;
                for (OrderableModelInfo mainPackage: mainPackageWithEqualNames){
                    String project = mainPackage.getProject().replace("/","");
                    String oldName = mainPackageName.substring(mainPackageName.indexOf(project) + project.length() + 1);
                    String oldPrefix = ".";
                    if(oldName.contains("."))
                        oldPrefix = "." + oldName.substring(0, oldName.lastIndexOf("."));
                    String newPref = mainPackage.getModelPath().substring(mainPackage.getModelPath().
                            indexOf(mainPackage.getRootName1() + "/" + project) +
                            (mainPackage.getRootName1() + "/" + project).length()).
                            replace("/", ".");
                    newPref = newPref.substring(0, newPref.lastIndexOf(oldPrefix));
                    if (!newPref.equals(""))
                        newPref = newPref.substring(newPref.lastIndexOf("."), newPref.length());
                    mainPackage.setModelName(project + newPref + "." + oldName);
                    int i = 1;
                }
            }
        }

        if (!allUnique)
            makeMainModelNamesUnique();
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
