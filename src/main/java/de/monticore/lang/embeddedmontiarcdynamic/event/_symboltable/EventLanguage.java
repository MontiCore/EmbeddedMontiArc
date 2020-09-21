/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event._symboltable;


import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.symboltable.resolving.CommonResolvingFilter;

public class EventLanguage extends EventLanguageTOP {
    public static final String FILE_ENDING = "event";

    public EventLanguage() {
        super("Event Definition Language", FILE_ENDING);
    }

    @Override
    protected EventModelLoader provideModelLoader() {
        return new EventModelLoader(this);
    }

    @Override
    protected void initResolvingFilters() {
        super.initResolvingFilters();
        this.addResolvingFilter(CommonResolvingFilter.create(MCTypeSymbol.KIND));
    }
}
