/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import com.google.common.collect.ImmutableSet;
import de.monticore.ast.ASTNode;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.ts.MCFieldSymbol;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.types2._symboltable.UnitNumberResolutionSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbol;
import de.monticore.lang.monticar.si._symboltable.SIUnitSymbol;
import de.monticore.modelloader.ModelingLanguageModelLoader;
import de.monticore.symboltable.resolving.CommonResolvingFilter;
import de.monticore.symboltable.types.JMethodSymbol;

import java.util.LinkedHashSet;

/**
 * The MontiArc Language
 *
 */
public class EmbeddedMontiViewLanguage extends EmbeddedMontiViewLanguageTOP {

  public static final String FILE_ENDING = "emv";

  public EmbeddedMontiViewLanguage() {
    super("Embedded MontiView Language", FILE_ENDING);
  }

  @Override
  protected void initResolvingFilters() {
    super.initResolvingFilters();
    // is done in generated TOP-language addResolver(new
    // CommonResolvingFilter<ViewComponentSymbol>(ViewComponentSymbol.class, ViewComponentSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(ViewComponentInstanceSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(ViewPortSymbol.KIND));
    addResolvingFilter(CommonResolvingFilter.create(ViewPortArraySymbol.KIND));
    addResolvingFilter(new EMAConnectorResolvingFilter<>(ViewConnectorSymbol.KIND));
    addResolvingFilter(new EMAConnectorResolvingFilter<>(ViewEffectorSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(ViewExpandedComponentInstanceSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(SIUnitSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(SIUnitRangesSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(MCTypeSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(MCFieldSymbol.KIND));
    addResolvingFilter(new CommonResolvingFilter<>(JMethodSymbol.KIND));
    addResolvingFilter(CommonResolvingFilter.create(ResolutionDeclarationSymbol.KIND));
    addResolvingFilter(CommonResolvingFilter.create(UnitNumberResolutionSymbol.KIND));
    setModelNameCalculator(new EmbeddedMontiArcModelNameCalculator());
  }

  /**
   * @see de.monticore.CommonModelingLanguage#provideModelLoader()
   */
  @Override
  protected ModelingLanguageModelLoader<? extends ASTNode> provideModelLoader() {
    return new EmbeddedMontiViewModelLoader(this);
  }
}
