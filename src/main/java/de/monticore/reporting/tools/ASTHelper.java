package de.monticore.reporting.tools;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.reporting.helper.CommonModelInfo;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Paths;

public class ASTHelper {

    public static void setTestResultInfo(CommonModelInfo model) {
        Log.enableFailQuick(false);
        Log.getFindings().clear();

        String fileName = model.getModelFileAsString();
        String fileType = fileName.substring(fileName.lastIndexOf(".") + 1, fileName.length()).toUpperCase();
        model.setFileType(fileType);
        ASTEMACompilationUnit ast = null;
        EMAComponentSymbol resolvedAST = null;

        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        try {
            model.addErrorMessage("[INFO] do Parser Test <br>=========================");
            ast = parser.parse(fileName).orElse(null);
        } catch (Exception e) {
            int i = 1;
        }
        boolean parsingSuccessful = ast != null;

        model.addErrorMessage(parsingSuccessful ? "[INFO] Parser Test success<br>" : "[ERROR] Parser Test failed");
        if(!parsingSuccessful){
            CustomPrinter.println("ERROR. Parser Test failed");
            for (Finding finding : Log.getFindings())
                model.addErrorMessage(finding.toString());
        }

        model.setParsed(parsingSuccessful?1:-1);
        if(parsingSuccessful) {
            model.setUnresolvedAST(ast);
            String PackageName = Joiners.DOT.join(ast.getPackageList());
            String FileName = fileName.substring(fileName.replace("\\", "/").lastIndexOf("/") + 1, fileName.length());
            String modelPath = fileName.substring(0, fileName.length() - (PackageName + "/" + FileName).length()); // package name + File name
            String modelName = PackageName + "." + FileName.replace(".emam", "").replace(".ema", "");
            String qualifiedName = FileName.replace(".emam", "").replace(".ema", "");
            qualifiedName = PackageName + "." + ("" + qualifiedName.charAt(0)).toLowerCase() + qualifiedName.substring(1, qualifiedName.length());

            model.setModelName(modelName);
            model.setModelPath(modelPath);
            model.setQualifiedName(qualifiedName);

            try {
                Log.getFindings().clear();
                model.addErrorMessage("[INFO] do Resolve Test<br>=========================");
                resolvedAST = getAstNode(modelPath, modelName, fileType);
                model.setResolvedAST(resolvedAST);
                model.setResolvedASTNode((ASTEmbeddedMontiArcNode) resolvedAST.getAstNode().get());
                model.setResolved(1);
                model.addErrorMessage("[INFO] Resolve Test success<br>");
            } catch (CouldNotResolveException e) {
                CustomPrinter.println("ERROR. Resolve Test failed");
                model.setResolved(-1);
                model.addErrorMessage("[ERROR] Resolve Test failed");
                for (Finding finding : Log.getFindings())
                    model.addErrorMessage(finding.toString());
            } catch (Exception e) {
                CustomPrinter.println("ERROR. Something went wrong");
                model.setResolved(-1);
                model.addErrorMessage("[ERROR] Something went wrong");
            }
        } else {
            String FileName = fileName.substring(fileName.replace("\\", "/").lastIndexOf("/") + 1, fileName.length());
            model.setModelName(FileName);
        }

    }

    public static EMAComponentSymbol getAstNode(String modelPath, String model, String fileType) throws CouldNotResolveException {
        try {
            Scope symTab = createSymTab(fileType, modelPath);
            EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(model, EMAComponentSymbol.KIND).orElse(null);
            return comp;
        } catch (Throwable  e) {
            throw new CouldNotResolveException();
        }
    }

    public static EMAComponentSymbol getAstNode(String file) throws CouldNotResolveException, IOException {
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMACompilationUnit ast = parser.parse(file).orElse(null);
        String fileType = file.substring(file.lastIndexOf(".") + 1, file.length()).toUpperCase();
        String PackageName = Joiners.DOT.join(ast.getPackageList());
        String FileName = file.substring(file.replace("\\", "/").lastIndexOf("/") + 1, file.length());
        String modelPath = file.substring(0, file.length() - (PackageName + "/" + FileName).length()); // package name + File name
        String modelName = PackageName + "." + FileName.replace(".emam", "").replace(".ema", "");
        return getAstNode(modelPath, modelName, fileType);
    }

    public static Scope createSymTab(String fileType, String... modelPath) {
        ModelingLanguageFamily fam = getModelingLanguageFamily(fileType);
        ModelPath mp = new ModelPath();
        for (String m : modelPath)
            mp.addEntry(Paths.get(m));
        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        return scope;
    }

    private static ModelingLanguageFamily getModelingLanguageFamily(String fileType) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        if (fileType.equals("EMAM"))
            fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
        else if (fileType.equals("EMA"))
            fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
        else
            Log.error("Unknown file type: " + fileType);
        fam.addModelingLanguage(new StreamLanguage());
        fam.addModelingLanguage(new StructLanguage());
        return fam;
    }
}
