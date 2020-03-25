/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt;

import de.monticore.ModelingLanguageFamily;
import de.monticore.cocos.helper.Assert;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.mathopt._ast.ASTMathOptCompilationUnit;
import de.monticore.lang.mathopt._ast.ASTMathOptNode;
import de.monticore.lang.mathopt._cocos.MathOptCoCoChecker;
import de.monticore.lang.mathopt._parser.MathOptParser;
import de.monticore.lang.mathopt._symboltable.MathOptLanguage;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayDeque;
import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.assertEquals;

/**
 * Abstract checker class to check MontiMathOpt models
 *
 */
public abstract class AbstractMathOptChecker {

    // override fields
    protected GlobalScope globalScope;
    protected MathOptParser parser = new MathOptParser();

    abstract protected MathOptCoCoChecker getChecker();

    protected void testModelForErrors(String model, Collection<Finding> expectedErrors) {
        MathOptCoCoChecker checker = getChecker();
        ASTMathOptCompilationUnit root = loadModel(model);
        checker.checkAll(root);
        Assert.assertEqualErrorCounts(expectedErrors, Log.getFindings());
        Assert.assertErrorMsg(expectedErrors, Log.getFindings());
    }

    protected void testModelNoErrors(String model) {
        MathOptCoCoChecker checker = getChecker();
        ASTMathOptCompilationUnit root = loadModel(model);
        checker.checkAll(root);
        assertEquals(0, Log.getFindings().stream().filter(f -> f.isError()).count());
    }

    protected ASTMathOptCompilationUnit loadModel(String modelFullQualifiedFilename) {
        Path model = Paths.get(modelFullQualifiedFilename);
        // get scope
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        MathOptLanguage lang = new MathOptLanguage();
        fam.addModelingLanguage(lang);
        final ModelPath mp = new ModelPath(model.toAbsolutePath());
        this.globalScope = new GlobalScope(mp, fam);
        // load model
        try {
            Optional<ASTMathOptCompilationUnit> root = parser.parse(model.toString());
            if (root.isPresent()) {
                // create Symboltable
                Optional<MathOptSymbolTableCreator> stc = getSymbolTableCreator(root.get(), Optional.of(globalScope));
                if (stc.isPresent()) {
                    stc.get().createFromAST(root.get().getMathCompilationUnit());
                }
                return root.get();
            }
        } catch (RecognitionException | IOException e) {
            e.printStackTrace();
        }
        throw new RuntimeException("Error during loading of model " + modelFullQualifiedFilename + ".");
    }

    public Optional<MathOptSymbolTableCreator> getSymbolTableCreator(ASTMathOptNode astNode, Optional<MutableScope> scopeOpt) {
        // return value
        Optional<MathOptSymbolTableCreator> stc = Optional.empty();
        // get language
        MathOptLanguage lang = new MathOptLanguage();
        ResolvingConfiguration resolvingConfiguration = new ResolvingConfiguration();
        resolvingConfiguration.addDefaultFilters(lang.getResolvingFilters());
        // get symbol table creator
        if (scopeOpt.isPresent())
            stc = lang.getSymbolTableCreator(resolvingConfiguration, scopeOpt.get());
        else
            stc = Optional.of(new MathOptSymbolTableCreator(resolvingConfiguration, new ArrayDeque<>()));

        return stc;
    }
}
