/* (c) https://github.com/MontiCore/monticore */
package de.monticore.reporting.tools;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEmbeddedMontiArcNode;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.EmbeddedMontiArcMathLanguage;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl._symboltable.EMADLLanguage;
import de.monticore.lang.monticar.enumlang._symboltable.EnumLangLanguage;
import de.monticore.lang.monticar.stream._symboltable.StreamLanguage;
import de.monticore.lang.monticar.streamunits._symboltable.StreamUnitsLanguage;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.reporting.helper.CommonModelInfo;
import de.monticore.reporting.helper.TimedTask;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.concurrent.*;

public class ASTHelper {

    public static void setTestResultInfo(CommonModelInfo model, int timeout) {
        Log.init();
        Log.enableFailQuick(false);
        Log.getFindings().clear();

        String fileName = model.getModelFileAsString();
        String fileType = fileName.substring(fileName.lastIndexOf(".") + 1, fileName.length()).toUpperCase();
        model.setFileType(fileType);
        ASTEMACompilationUnit ast = null;

        try {
            model.addErrorMessage("[INFO] do Parser Test <br>=========================");
            ast = getParsed(fileName, timeout);
        } catch (ParsingTimeOutException e) {
            CustomPrinter.println("ERROR. Parsing Test timed out");
            model.setParsed(-2);
            model.addErrorMessage("[ERROR] Parsing Test timed out");
            for (Finding finding : Log.getFindings())
                model.addErrorMessage(finding.toString());
        } catch (CouldNotParseException e) {
            CustomPrinter.println("ERROR. Parsing Test failed");
            model.setParsed(-1);
            model.addErrorMessage("[ERROR] Parsing Test failed");
            model.addErrorMessage(e.toString());
            for (Finding finding : Log.getFindings())
                model.addErrorMessage(finding.toString());
        }
        boolean parsingSuccessful = ast != null;
        if (parsingSuccessful) {
            model.setParsed(1);
            model.addErrorMessage("[INFO] Parser Test success<br>");
        }

        if (parsingSuccessful) {
            model.setUnresolvedAST(ast);
            String PackageName = Joiners.DOT.join(ast.getPackageList());
            if (!PackageName.equals("")) PackageName = PackageName + ".";
            String FileName = fileName.substring(fileName.replace("\\", "/").lastIndexOf("/") + 1, fileName.length());
            String modelPath = fileName.substring(0, fileName.length() - (PackageName + "/" + FileName).length()); // package name + File name
            String qualifiedName = FileName.substring(0, FileName.lastIndexOf("."));
            String modelName = PackageName + qualifiedName;
            qualifiedName = PackageName + ("" + qualifiedName.charAt(0)).toLowerCase() + qualifiedName.substring(1, qualifiedName.length());

            model.setModelName(modelName);
            model.setModelPath(modelPath);
            model.setQualifiedName(qualifiedName);

            try {
                Log.getFindings().clear();
                model.addErrorMessage("[INFO] do Resolve Test<br>=========================");
                EMAComponentSymbol resolvedAST = getAstNode(modelPath, modelName, fileType, timeout);
                model.setResolvedAST(resolvedAST);
                model.setResolvedASTNode((ASTEmbeddedMontiArcNode) resolvedAST.getAstNode().get());
                model.setResolved(1);
                model.addErrorMessage("[INFO] Resolve Test success<br>");
            } catch (CouldNotResolveException e) {
                CustomPrinter.println("ERROR. Resolve Test failed");
                model.setResolved(-1);
                model.addErrorMessage("[ERROR] Resolve Test failed");
                model.addErrorMessage(e.toString());
                for (Finding finding : Log.getFindings())
                    model.addErrorMessage(finding.toString());
            } catch (ResolveTimeOutException e) {
                CustomPrinter.println("ERROR. Resolve Test timed out");
                model.setResolved(-2);
                model.addErrorMessage("[ERROR] Resolve Test timed out");
                for (Finding finding : Log.getFindings())
                    model.addErrorMessage(finding.toString());
            } catch (Exception e) {
                CustomPrinter.println("ERROR. Resolve Test failed");
                model.setResolved(-1);
                model.addErrorMessage("[ERROR] Resolve Test failed");
                model.addErrorMessage(e.toString());
                for (Finding finding : Log.getFindings())
                    model.addErrorMessage(finding.toString());
            }
        } else {
            String FileName = fileName.substring(fileName.replace("\\", "/").lastIndexOf("/") + 1, fileName.length());
            model.setModelName(FileName);
        }

    }

    private static ExecutorService service = Executors.newSingleThreadExecutor();
    private static ScheduledExecutorService canceller = Executors.newSingleThreadScheduledExecutor();

    public static ASTEMACompilationUnit getParsed(String fileName, int timeout) throws CouldNotParseException, ParsingTimeOutException {
//        ExecutorService executor = Executors.newSingleThreadExecutor();
        Callable<ASTEMACompilationUnit> task = () -> {
            EMADLParser parser = new EMADLParser();
            ASTEMACompilationUnit ast = parser.parse(fileName).orElse(null);
            if (ast == null) throw new CouldNotParseException("");
            return ast;
        };

        Future<ASTEMACompilationUnit> future = TimedTask.executeTask(task, timeout);
        try {
            ASTEMACompilationUnit res = future.get();
            future.cancel(true);
            return res;
        } catch (CancellationException ex) {
            future.cancel(true);
            throw new ParsingTimeOutException();
        } catch (InterruptedException e) {
            future.cancel(true);
            throw new ParsingTimeOutException();
        } catch (Throwable e) {
            future.cancel(true);
            throw new CouldNotParseException(e.toString());
        }
    }

    public static EMAComponentSymbol getAstNode(String modelPath, String model, String fileType, int timeout) throws CouldNotResolveException, ResolveTimeOutException {
//        ExecutorService executor = Executors.newSingleThreadExecutor();

        Callable<EMAComponentSymbol> task = () -> {
            Scope symTab = createSymTab(fileType, modelPath);
            EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(model, EMAComponentSymbol.KIND).orElse(null);
            return comp;
        };

        Future<EMAComponentSymbol> future = TimedTask.executeTask(task, timeout);
        try {
            EMAComponentSymbol res = future.get();
            future.cancel(true);
            return res;
        } catch (CancellationException ex) {
            future.cancel(true);
            throw new ResolveTimeOutException();
        } catch (InterruptedException e) {
            future.cancel(true);
            throw new ResolveTimeOutException();
        } catch (Throwable e) {
            future.cancel(true);
            throw new CouldNotResolveException(e.toString());
        }
    }

    public static Scope createSymTab(String fileType, String... modelPath) {
        ModelingLanguageFamily fam = getModelingLanguageFamily(fileType);
        ModelPath mp = new ModelPath();
        for (String m : modelPath)
            mp.addEntry(Paths.get(m));
        GlobalScope scope = new GlobalScope(mp, fam);
        de.monticore.lang.monticar.Utils.addBuiltInTypes(scope);
        Log.init();
        Log.enableFailQuick(false);
        return scope;
    }

    private static ModelingLanguageFamily getModelingLanguageFamily(String fileType) {
        ConstantPortHelper.resetLastID();
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
//        fam.addModelingLanguage(new StreamUnitsLanguage());
//        fam.addModelingLanguage(new StructLanguage());
//        fam.addModelingLanguage(new EnumLangLanguage());
        if (fileType.equals("EMAM"))
            fam.addModelingLanguage(new EmbeddedMontiArcMathLanguage());
        else if (fileType.equals("EMA"))
            fam.addModelingLanguage(new EmbeddedMontiArcLanguage());
        else if (fileType.equals("M"))
            fam.addModelingLanguage(new MathLanguage());
        else if (fileType.equals("EMADL"))
            fam.addModelingLanguage(new EMADLLanguage());
        else
            Log.error("Unknown file type: " + fileType);
        fam.addModelingLanguage(new StreamLanguage());
        fam.addModelingLanguage(new StructLanguage());
        return fam;
    }
}
