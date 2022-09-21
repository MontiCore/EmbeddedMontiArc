/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.EmbeddingModelingLanguage;
import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter.PortArraySymbol2MathVariableDeclarationSymbolTypeFilter;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.adapter.ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicLanguage;
import de.monticore.lang.mathopt._symboltable.MathOptLanguage;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchLanguage;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.lang.monticar.emadl.adapter.PortArraySymbol2IODeclarationSymbolTypeFilter;
import de.monticore.lang.monticar.emadl.modularcnn.composer.ArchitectureNode;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.resolving.ResolvingFilter;

import java.util.*;

public class EMADLLanguage extends EmbeddingModelingLanguage {

    public static final String FILE_ENDING = "emadl";
    private String customFilesPath = "";
    private String backend = "";
    private String pythonPath = "";

    public static final EmbeddedMontiArcDynamicLanguage HOST_LANGUAGE = new EmbeddedMontiArcDynamicLanguage();
    public static final CNNArchLanguage CNNARCH_LANGUAGE = new CNNArchLanguage();
    public static final MathOptLanguage MATH_LANGUAGE = new MathOptLanguage();

    private ArrayList<ArchitectureNode> architectureNodes = new ArrayList<>();

    public EMADLLanguage() {
        super("Embedded MontiArc Deep Learning Language",
                FILE_ENDING,
                HOST_LANGUAGE,
                Arrays.asList(CNNARCH_LANGUAGE, MATH_LANGUAGE));
    }

    public EMADLLanguage(String customFilesPath, String pythonPath, String backend) {
        super("Embedded MontiArc Deep Learning Language",
                FILE_ENDING,
                HOST_LANGUAGE,
                Arrays.asList(CNNARCH_LANGUAGE, MATH_LANGUAGE));
        setBackend(backend);
        setCustomFilesPath(customFilesPath);
        setPythonPath(pythonPath);
    }

    private void setBackend(String backend){
        this.backend = backend;
    }

    public String getBackend(){
        return this.backend;
    }

    private void setCustomFilesPath(String customPythonFilesPath){
        this.customFilesPath = customPythonFilesPath;
    }

    public String getCustomFilesPath(){
        return this.customFilesPath;
    }

    private void setPythonPath (String pythonPath){ this.pythonPath = pythonPath; }

    public String getPythonPath (){ return this.pythonPath; }

    public ArrayList<ArchitectureNode> getArchitectureNodes() {
        return architectureNodes;
    }


    @Override
    public Collection<ResolvingFilter<? extends Symbol>> getResolvingFilters() {
        List<ResolvingFilter<? extends Symbol>> ret =
                new ArrayList<>(super.getResolvingFilters());
        ret.add(new PortArraySymbol2IODeclarationSymbolTypeFilter());
        //ret.add(new ResolutionDeclarationSymbol2ParameterSymbolTypeFilter());

        ret.add(new PortArraySymbol2MathVariableDeclarationSymbolTypeFilter());
        ret.add(new ResolutionDeclarationSymbol2MathVariableDeclarationTypeFilter());
        return ret;
    }


    @Override
    protected EMADLModelLoader provideModelLoader() {
        return new EMADLModelLoader(this);
    }

    @Override
    public MCConcreteParser getParser() {
        return new EMADLParser();
    }



    @Override
    public Optional<EMADLSymbolTableCreator> getSymbolTableCreator(ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
        return Optional.of(new EMADLSymbolTableCreator(
                resolvingConfiguration, enclosingScope,getCustomFilesPath(), getPythonPath(), getBackend(), getArchitectureNodes()));
    }

}
