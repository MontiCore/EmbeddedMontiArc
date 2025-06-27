package conflang;

import conflang._ast.ASTConfLangCompilationUnit;
import conflang._ast.ASTConfiguration;
import conflang._parser.ConfLangParser;
import conflang._symboltable.ConfLangLanguage;
import conflang._symboltable.ConfLangSymbolTableCreator;
import conflang._symboltable.ConfigurationScope;
import conflang.exception.ConfigurationNotFoundException;
import de.monticore.io.paths.ModelPath;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Optional;

/**
 *
 */
public class ConfLangFacade {

    /**
     * Symbol table of this configuration.
     */
    private ConfigurationScope artifactScope;

    /**
     * Hidden constructor. Instances of this class are created using the {@link #create(Path, String)} factory method.
     */
    private ConfLangFacade() {
    }

    private ConfLangFacade(Path modelPath, String model) {
        init(modelPath, model);
    }

    /**
     * Creates an instance of a {@link ConfLangFacade} based on a ConfLang configuration model.
     *
     * @param modelPath the path to the folder containing the configuration model.
     * @param modelName the name of the configuration model.
     * @return
     */
    public static ConfLangFacade create(Path modelPath, String modelName) {
        return new ConfLangFacade(modelPath, modelName);
    }

    /**
     * Returns the reference to the symbol table for this configuration.
     *
     * @return
     */
    public ConfigurationScope getArtifactScope() {
        return artifactScope;
    }

    private void init(Path modelPath, String model) {
        // GlobalScope confLangGlobalScope = new GlobalScope(new ModelPath(new Path[]{ modelPath }), new ConfLangLanguage());
        Optional<ASTConfLangCompilationUnit> confLangCompilationUnitOpt = parse(modelPath, model);
        if (!confLangCompilationUnitOpt.isPresent()) {
            throw new ConfigurationNotFoundException(String.format("No ConfLang model named %s available.", model));
        }
        ASTConfLangCompilationUnit confLangCompilationUnit = confLangCompilationUnitOpt.get();
        artifactScope = createSymbolTable(confLangCompilationUnit.getConfiguration());
    }

    private Optional<ASTConfLangCompilationUnit> parse(Path modelPath, String model) {
        Path path = Paths.get(modelPath.toString(), model);
        ConfLangParser parser = new ConfLangParser();
        Optional<ASTConfLangCompilationUnit> compilationUnit;

        try {
            compilationUnit = parser.parse(path.toString());
            return compilationUnit;
        } catch (IOException e) {
            return Optional.empty();
        }
    }

    private ConfigurationScope createSymbolTable(ASTConfiguration configuration) {
        final ModelPath modelPath = new ModelPath(Paths.get("src/test/resources/schemalang/cocos"));
        final ConfLangLanguage confLangLanguage = new ConfLangLanguage();
        final ResolvingConfiguration resolverConfiguration = new ResolvingConfiguration();
        resolverConfiguration.addDefaultFilters(confLangLanguage.getResolvingFilters());

        ConfLangSymbolTableCreator symbolTableCreator = new ConfLangSymbolTableCreator(resolverConfiguration,
                new GlobalScope(modelPath, confLangLanguage));
        ConfigurationScope confLangConfigurationScope = (ConfigurationScope) symbolTableCreator.createFromAST(configuration);
        return confLangConfigurationScope;
    }
}