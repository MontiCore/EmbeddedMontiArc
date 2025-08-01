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

    private Map<String, List<T>> mainPackageModels;
    private T notParsed, notResolved;
    private OrderableModelInfoCreator<T> creator;

    public OrderTestResults(OrderableModelInfoCreator<T> creator) {
        this.mainPackageModels = new HashMap<>();
        this.creator = creator;
        initErroredMainPackages();
    }

    private void initErroredMainPackages() {
        this.notParsed = creator.createNewInstance("");
        this.notResolved = creator.createNewInstance("");
        this.notParsed.setErrorResult(true);
        this.notResolved.setErrorResult(true);
        this.notParsed.setModelName("Parsing failed");
        this.notResolved.setModelName("Resolving failed");
        this.notParsed.setProject("Errored_Parsing");
        this.notResolved.setProject("Errored_Resolving");
        this.notParsed.setParsed(-1);
        this.notResolved.setResolved(-1);
    }

    public static<T extends OrderableModelInfo> Map<String, List<T>> orderTestResults(File root, Set<String> projectsToTest, List<T> testResults, OrderableModelInfoCreator<T> creator) {
        OrderTestResults<T> order = new OrderTestResults(creator);
        order.getMainPackageModels().put(allProjects, new LinkedList<>());
        order.orderTestResultsMain(allProjects, testResults, creator);

        for (File project : root.listFiles()) {
            if (project.isDirectory() && projectsToTest.contains(project.getName())) {
                List<T> projectTestResults = testResults.stream().filter(
                        t -> t.getProject().contains(project.getName())
                ).collect(Collectors.toList());
                order.getMainPackageModels().put(project.getName(), new LinkedList<>());
                order.orderTestResultsMain(project.getName(), projectTestResults, creator);
            }
        }
        return order.getMainPackageModels();
    }

    private void orderTestResultsMain(String projectName, List<T> testResults, OrderableModelInfoCreator<T> creator) {

        Map<String, Map<String, T>> maps = mapTestResults(testResults);
        Map<String, Map<String, List<T>>> mainPackages = new HashMap<>();
        initErroredMainPackages();

        for (T testResult : testResults) {
            if (testResult.getParsed() != 1)
                setErroredParent(testResult, this.notParsed);
            else if (testResult.getResolved() != 1)
                setErroredParent(testResult, this.notResolved);
            else {
                EMAComponentSymbol ast = testResult.getResolvedAST();
                Map<String, T> modelPathMap = maps.get(testResult.getModelPath());

                for (ASTSubComponent element : ((ASTComponent) ast.getAstNode().get()).getSubComponents()) {
                    testResult.setAtomic(false);
                    addChildToTestResult(testResult, ast, element, modelPathMap);
                }
            }

            if (testResult.getParsed() == 1) {
                String mainPackageName = getMainPackageName(testResult.getModelName());
                addTestResultToMainModelList(mainPackageName, testResult, mainPackages);
            }
        }

        if (notParsed.getChildren().size() > 0)
            this.mainPackageModels.get(projectName).add(notParsed);
        if (notResolved.getChildren().size() > 0)
            this.mainPackageModels.get(projectName).add(notResolved);

        addTestResultsToMainModels(mainPackages, projectName);
        makeMainModelNamesUnique(projectName);
        fixMainModelNames(projectName);
    }

    private void addTestResultsToMainModels(Map<String, Map<String, List<T>>> mainPackages, String projectName) {
        for (String modelPath : mainPackages.keySet()) {
            Map<String, List<T>> modelPathModels = mainPackages.get(modelPath);

            for (String mainPackage : modelPathModels.keySet()) {
                List<T> insideMainPackage = modelPathModels.get(mainPackage);

                T mainPackageResult = this.creator.createNewInstance(modelPath + "." + mainPackage);
                this.mainPackageModels.get(projectName).add(mainPackageResult);
                mainPackageResult.setMainPackage(true);

                for (OrderableModelInfo testResult : insideMainPackage) {
                    addChildToMainPackage(mainPackage, mainPackageResult, testResult);
                }
            }
        }
    }

    private void addTestResultToMainModelList(String mainPackageName, T testResult, Map<String, Map<String, List<T>>> mainPackages) {
        if (mainPackages.get(testResult.getModelPath()) == null)
            mainPackages.put(testResult.getModelPath(), new HashMap<>());
        Map<String, List<T>> modelPathModels = mainPackages.get(testResult.getModelPath());

        if (modelPathModels.get(mainPackageName) == null)
            modelPathModels.put(mainPackageName, new LinkedList<>());
        modelPathModels.get(mainPackageName).add(testResult);
    }

    private String getMainPackageName(String modelName) {
        if (modelName.startsWith("."))
            modelName = modelName.substring(1);
        String mainPackageName = null;
        try {
            mainPackageName = modelName.substring(0, modelName.indexOf("."));
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (mainPackageName == null)
            mainPackageName = "No Package";
        else if (mainPackageName.equals(""))
            mainPackageName = modelName.substring(1).substring(modelName.substring(1).indexOf("."));
        return mainPackageName;
    }

    private void addChildToTestResult(T testResult, EMAComponentSymbol ast, ASTSubComponent element, Map<String, T> modelPathMap) {
        String childName = getNameOfImportedSubComponent(ast, element);
        OrderableModelInfo child = modelPathMap.get(childName);
        if (child != null) {
            String referencedName = element.getInstancesList().get(0).getName();
            ChildElement childElement = new ChildElement(referencedName, child);
            testResult.addChild(childElement);
            child.addParent(testResult);
        }
    }

    private String getNameOfImportedSubComponent(EMAComponentSymbol ast, ASTSubComponent element) {
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
        return name;
    }

    private void addChildToMainPackage(String mainPackage, T mainPackageResult, OrderableModelInfo testResult) {
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
