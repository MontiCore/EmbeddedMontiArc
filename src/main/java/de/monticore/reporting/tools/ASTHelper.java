package de.monticore.reporting.tools;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.LogConfig;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._ast.ASTEMAMCompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.reporting.cocoReport.helper.CheckCoCoResult;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Paths;

public class ASTHelper {

    public static void setTestResultInfo(CheckCoCoResult testResult) {
        Log.enableFailQuick(false);
        Log.getFindings().clear();

        String fileName = testResult.getPathToFile();
        String fileType = fileName.substring(fileName.lastIndexOf(".") + 1, fileName.length()).toUpperCase();
        testResult.setFileType(fileType);
        boolean parse = false;
        ASTEMAMCompilationUnit ast = null;
        ASTEmbeddedMontiArcNode resolvedAST = null;

        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        try {
            testResult.addErrorMessage("[INFO] do Parser Test <br>=========================");
            ast = parser.parse(fileName).orElse(null);
        } catch (Exception e) {}
        parse = ast != null;

        testResult.addErrorMessage(parse ? "[INFO] Parser Test success<br>" : "[ERROR] Parser Test failed");
        if(!parse){
            CustomPrinter.println("ERROR. Parser Test failed");
            for (Finding finding : Log.getFindings())
                testResult.addErrorMessage(finding.getMsg());
        }

        testResult.setParsed(parse?1:-1);
        if(parse) {
            testResult.setCompilationUnit(ast);
            String PackageName = Joiners.DOT.join(ast.getEMACompilationUnit().getPackage());
            String FileName = fileName.substring(fileName.replace("\\", "/").lastIndexOf("/") + 1, fileName.length());
            String modelPath = fileName.substring(0, fileName.length() - (PackageName + "/" + FileName).length()); // package name + File name
            String modelName = PackageName + "." + FileName.replace(".emam", "").replace(".ema", "");
            String qualifiedName = FileName.replace(".emam", "").replace(".ema", "");
            qualifiedName = PackageName + "." + ("" + qualifiedName.charAt(0)).toLowerCase() + qualifiedName.substring(1, qualifiedName.length());

            testResult.setModelName(modelName);
            testResult.setModelPath(modelPath);
            testResult.setQualifiedName(qualifiedName);

            try {
                Log.getFindings().clear();
                testResult.addErrorMessage("[INFO] do Resolve Test<br>=========================");
                resolvedAST = getAstNode(modelPath, modelName, fileType);
                testResult.setResolvedAst(resolvedAST);
                testResult.setResolved(1);
                testResult.addErrorMessage("[INFO] Resolve Test success<br>");
            } catch (CouldNotResolveException e) {
                CustomPrinter.println("ERROR. Resolve Test failed");
                testResult.setResolved(-1);
                testResult.addErrorMessage("[ERROR] Resolve Test failed");
                for (Finding finding : Log.getFindings())
                    testResult.addErrorMessage(finding.getMsg());
            } catch (Exception e) {
                CustomPrinter.println("ERROR. Something went wrong");
                testResult.setResolved(-1);
                testResult.addErrorMessage("[ERROR] Something went wrong");
            }
        } else {
            String FileName = fileName.substring(fileName.replace("\\", "/").lastIndexOf("/") + 1, fileName.length());
            testResult.setModelName(FileName);
        }

    }

    public static ASTEmbeddedMontiArcNode getAstNode(String modelPath, String model, String fileType) throws CouldNotResolveException {
        try {
            Scope symTab = createSymTab(fileType, modelPath);
            ComponentSymbol comp = symTab.<ComponentSymbol>resolve(
                    model, ComponentSymbol.KIND).orElse(null);

            return (ASTEmbeddedMontiArcNode) comp.getAstNode().get();
        } catch (Exception e) {
            throw new CouldNotResolveException();
        }
    }

    public static ASTEmbeddedMontiArcNode getAstNode(String file) throws CouldNotResolveException, IOException {
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMAMCompilationUnit ast = parser.parse(file).orElse(null);
        String fileType = file.substring(file.lastIndexOf(".") + 1, file.length()).toUpperCase();
        String PackageName = Joiners.DOT.join(ast.getEMACompilationUnit().getPackage());
        String FileName = file.substring(file.replace("\\", "/").lastIndexOf("/") + 1, file.length());
        String modelPath = file.substring(0, file.length() - (PackageName + "/" + FileName).length()); // package name + File name
        String modelName = PackageName + "." + FileName.replace(".emam", "").replace(".ema", "");
        return getAstNode(modelPath, modelName, fileType);
    }

    public static Scope createSymTab(String fileType, String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        if (fileType.equals("EMAM"))
            fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
        else if (fileType.equals("EMA"))
            fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
        else
            Log.error("Unknown file type: " + fileType);
        fam.addModelingLanguage(new StreamLanguage());
        fam.addModelingLanguage(new StructLanguage());
        final ModelPath mp = new ModelPath();
        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }
        GlobalScope scope = new GlobalScope(mp, fam);

        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        LogConfig.init();
        return scope;
    }
}
