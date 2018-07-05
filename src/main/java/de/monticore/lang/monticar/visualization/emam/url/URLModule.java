package de.monticore.lang.monticar.visualization.emam.url;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;

public class URLModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<URLContribution> urls =
                Multibinder.newSetBinder(binder(), URLContribution.class);

        urls.addBinding().to(VisualizationURL.class);
        urls.addBinding().to(MathPrettyPrinterURL.class);

        bind(URLService.class).to(URLServiceImpl.class);
    }
}
