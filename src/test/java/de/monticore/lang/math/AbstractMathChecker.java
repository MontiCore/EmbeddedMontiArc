/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.ModelingLanguageFamily;
import de.monticore.cocos.helper.Assert;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math._ast.ASTMathCompilationUnit;
import de.monticore.lang.math._cocos.MathCoCoChecker;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.math._symboltable.MathLanguage;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Finding;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.Optional;

import static org.junit.Assert.assertEquals;

/**
 * Created by Tobias PC on 30.12.2016.
 */
public abstract class AbstractMathChecker {
    private final MathLanguage mathlang = new MathLanguage();
    MathParser parser = new MathParser();
    private GlobalScope globalScope;


    public AbstractMathChecker() {
    }

    abstract protected MathCoCoChecker getChecker();

    protected void testModelForErrors(String model, Collection<Finding> expectedErrors) {
        MathCoCoChecker checker = getChecker();
        ASTMathCompilationUnit root = loadModel(model);
        checker.checkAll(root);
        Assert.assertEqualErrorCounts(expectedErrors, Log.getFindings());
        Assert.assertErrorMsg(expectedErrors, Log.getFindings());
    }

    protected void testModelNoErrors(String model) {
        MathCoCoChecker checker = getChecker();
        ASTMathCompilationUnit root = loadModel(model);
        checker.checkAll(root);
        assertEquals(0, Log.getFindings().stream().filter(f -> f.isError()).count());
    }

    protected ASTMathCompilationUnit loadModel(String modelFullQualifiedFilename) {
        Path model = Paths.get(modelFullQualifiedFilename);

        try {
            Optional<ASTMathCompilationUnit> root = parser.parse(model.toString());
            if (root.isPresent()) {
                // create Symboltable
                ModelingLanguageFamily fam = new ModelingLanguageFamily();
                fam.addModelingLanguage(new MathLanguage());
                final ModelPath mp = new ModelPath(model.toAbsolutePath());
                this.globalScope = new GlobalScope(mp, fam);

                ResolvingConfiguration ResolvingConfiguration = new ResolvingConfiguration();
                ResolvingConfiguration.addDefaultFilters(mathlang.getResolvingFilters ());
                Optional<MathSymbolTableCreator> stc = mathlang.getSymbolTableCreator(ResolvingConfiguration, globalScope);
                if (stc.isPresent()) {
                    stc.get().createFromAST(root.get());
                }
                return root.get();
            }
        } catch (RecognitionException | IOException e) {
            e.printStackTrace();
        }
        throw new RuntimeException("Error during loading of model " + modelFullQualifiedFilename + ".");
    }
}
