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
            for (Finding finding : Log.getFindings())
                model.addErrorMessage(finding.toString());
        }
        boolean parsingSuccessful = ast != null;
        if (parsingSuccessful) {
            model.addErrorMessage("[INFO] Parser Test success<br>");
        }

        model.setParsed(parsingSuccessful ? 1 : -1);
        if (parsingSuccessful) {
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
                EMAComponentSymbol resolvedAST = getAstNode(modelPath, modelName, fileType, timeout);
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
            } catch (ResolveTimeOutException e) {
                CustomPrinter.println("ERROR. Resolve Test timed out");
                model.setResolved(-2);
                model.addErrorMessage("[ERROR] Resolve Test timed out");
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

    public static ASTEMACompilationUnit getParsed(String fileName, int timeout) throws CouldNotParseException, ParsingTimeOutException {
        ExecutorService executor = Executors.newSingleThreadExecutor();
        Callable<ASTEMACompilationUnit> task = new Callable<ASTEMACompilationUnit>() {
            public ASTEMACompilationUnit call() throws IOException {
                EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
                return parser.parse(fileName).orElse(null);
            }
        };

        Future<ASTEMACompilationUnit> future = executor.submit(task);
        try {
            return future.get(timeout, TimeUnit.SECONDS);
        } catch (TimeoutException ex) {
            future.cancel(true);
            executor.shutdown();
            executor.shutdownNow();
            throw new ParsingTimeOutException();
        } catch (InterruptedException e) {
            future.cancel(true);
            executor.shutdown();
            executor.shutdownNow();
            throw new ParsingTimeOutException();
        } catch (Throwable e) {
            future.cancel(true);
            throw new CouldNotParseException();
        }
    }

    public static EMAComponentSymbol getAstNode(String modelPath, String model, String fileType, int timeout) throws CouldNotResolveException, ResolveTimeOutException {
        ExecutorService executor = Executors.newSingleThreadExecutor();
        Callable<EMAComponentSymbol> task = new Callable<EMAComponentSymbol>() {
            public EMAComponentSymbol call() {
                Scope symTab = createSymTab(fileType, modelPath);
                EMAComponentSymbol comp = symTab.<EMAComponentSymbol>resolve(model, EMAComponentSymbol.KIND).orElse(null);
                return comp;
            }
        };

        Future<EMAComponentSymbol> future = executor.submit(task);
        try {
            return future.get(timeout, TimeUnit.SECONDS);
        } catch (TimeoutException ex) {
            future.cancel(true);
            executor.shutdown();
            executor.shutdownNow();
            throw new ResolveTimeOutException();
        } catch (InterruptedException e) {
            future.cancel(true);
            executor.shutdown();
            executor.shutdownNow();
            throw new ResolveTimeOutException();
        } catch (Throwable e) {
            future.cancel(true);
            throw new CouldNotResolveException();
        }
    }

    public static EMAComponentSymbol getAstNode(String file, int timeout) throws CouldNotResolveException, IOException, ResolveTimeOutException {
        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        ASTEMACompilationUnit ast = parser.parse(file).orElse(null);
        String fileType = file.substring(file.lastIndexOf(".") + 1, file.length()).toUpperCase();
        String PackageName = Joiners.DOT.join(ast.getPackageList());
        String FileName = file.substring(file.replace("\\", "/").lastIndexOf("/") + 1, file.length());
        String modelPath = file.substring(0, file.length() - (PackageName + "/" + FileName).length()); // package name + File name
        String modelName = PackageName + "." + FileName.replace(".emam", "").replace(".ema", "");
        return getAstNode(modelPath, modelName, fileType, timeout);
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
