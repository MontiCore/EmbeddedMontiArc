/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.order;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.reporting.helper.OrderableModelInfo;
import de.monticore.reporting.helper.OrderableModelInfoCreator;
import de.monticore.symboltable.ImportStatement;
import de.monticore.types.types._ast.ASTSimpleReferenceType;

import java.io.File;
import java.util.*;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

public class OrderTestResults<T extends OrderableModelInfo> {

    public static final String allProjects = "allProjects";

    private Map<String, List<T>> mainPackageModels = new HashMap<>();

    public void orderTestResults(File root, Set<String> projectsToTest, List<T> testResults, OrderableModelInfoCreator<T> creator) {
        mainPackageModels.put(allProjects, new LinkedList<>());
        orderTestResultsMain(allProjects, testResults, creator);

        for (File project : root.listFiles()) {
            if (project.isDirectory() && projectsToTest.contains(project.getName())) {
                List<T> projectTestResults = testResults.stream().filter(
                        t -> t.getProject().contains(project.getName())
                ).collect(Collectors.toList());
                mainPackageModels.put(project.getName(), new LinkedList<>());
                orderTestResultsMain(project.getName(), projectTestResults, creator);
            }
        }
    }

    private void orderTestResultsMain(String projectName, List<T> testResults, OrderableModelInfoCreator<T> creator) {

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
                setErroredParent(testResult, notParsed);
            } else if (testResult.getResolved() != 1) {
                setErroredParent(testResult, notResolved);
            } else {
                EMAComponentSymbol ast = testResult.getResolvedAST();
                String modelPath = testResult.getModelPath();
                Map<String, T> modelPathMap = maps.get(modelPath);

                for (ASTSubComponent element : ((ASTComponent) ast.getAstNode().get()).getSubComponents()) {
                    testResult.setAtomic(false);
                    String type = ((ASTSimpleReferenceType) element.getType()).getName(0);
                    String name = "";
                    for (ImportStatement importStatement : ast.getImports()) {
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

                if (mainPackage == null)
                    mainPackage = "No Package";
                else if (mainPackage.equals(""))
                    mainPackage = modelName.substring(1).substring(modelName.substring(1).indexOf("."));

                if (mainPackages.get(testResult.getModelPath()) == null)
                    mainPackages.put(testResult.getModelPath(), new HashMap<>());
                Map<String, List<T>> modelPathModels = mainPackages.get(testResult.getModelPath());

                if (modelPathModels.get(mainPackage) == null)
                    modelPathModels.put(mainPackage, new LinkedList<>());
                modelPathModels.get(mainPackage).add(testResult);
            }
        }

        if (notParsed.getChildren().size() > 0)
            mainPackageModels.get(projectName).add(notParsed);
        if (notResolved.getChildren().size() > 0)
            mainPackageModels.get(projectName).add(notResolved);

        for (String modelPath : mainPackages.keySet()) {
            Map<String, List<T>> modelPathModels = mainPackages.get(modelPath);

            for (String mainPackage : modelPathModels.keySet()) {
                List<T> insideMainPackage = modelPathModels.get(mainPackage);

                T mainPackageResult = creator.createNewInstance(modelPath + "." + mainPackage);
                mainPackageModels.get(projectName).add(mainPackageResult);
                mainPackageResult.setMainPackage(true);

                for (OrderableModelInfo testResult : insideMainPackage) {
                    if (mainPackageResult.getProject().equals(""))
                        mainPackageResult.setProject(testResult.getProject());
                    mainPackageResult.addChild(new ChildElement(testResult.getModelName(), testResult));
                    if (mainPackageResult.getRootFile() == null)
                        mainPackageResult.setRootFile(testResult.getRootFile());
                    if (mainPackageResult.getRootName().equals(""))
                        mainPackageResult.setRootName(testResult.getRootName());
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
        makeMainModelNamesUnique(projectName);
        fixMainModelNames(projectName);
    }

    private void setErroredParent(T child, T parent) {
        if (parent.getRootName().equals("") && !child.getRootName().contains(OrderableModelInfo.erroredString))
            parent.setRootName(OrderableModelInfo.erroredString + "_" + child.getRootName());
        else if(parent.getRootName().equals("")) parent.setRootName(child.getRootName());
        child.setRootName(parent.getRootName());
        parent.addChild(new ChildElement("", child));
        child.addParent(parent);
    }

    private void fixMainModelNames(String projectName) {
        for (OrderableModelInfo mainModel : this.mainPackageModels.get(projectName))
            mainModel.setModelName(mainModel.getModelName().replaceFirst(Pattern.quote("."), " - "));
    }

    private void makeMainModelNamesUnique(String projectName) {
        boolean allUnique = true;
        Map<String, List<OrderableModelInfo>> mainPackagesMap = new HashMap<>();
        for (OrderableModelInfo mainPackage : this.mainPackageModels.get(projectName)) {
            String name = mainPackage.getModelName();
            if (mainPackagesMap.get(name) == null)
                mainPackagesMap.put(name, new LinkedList<>());
            mainPackagesMap.get(name).add(mainPackage);
        }

        for (String mainPackageName : mainPackagesMap.keySet()) {
            List<OrderableModelInfo> mainPackageWithEqualNames = mainPackagesMap.get(mainPackageName);
            if (mainPackageWithEqualNames.size() > 1) {
                allUnique = false;
                for (OrderableModelInfo mainPackage : mainPackageWithEqualNames) {
                    String project = mainPackage.getProject().replace("/", "");
                    String oldName = mainPackageName.substring(mainPackageName.indexOf(project) + project.length() + 1);
                    String oldPrefix = ".";
                    if (oldName.contains("."))
                        oldPrefix = "." + oldName.substring(0, oldName.lastIndexOf("."));
                    String newPref = mainPackage.getModelPath().substring(mainPackage.getModelPath().
                            indexOf(mainPackage.getRootName() + "/" + project) +
                            (mainPackage.getRootName() + "/" + project).length()).
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
            makeMainModelNamesUnique(projectName);
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

    public Map<String, List<T>> getMainPackageModels() {
        return mainPackageModels;
    }
}
