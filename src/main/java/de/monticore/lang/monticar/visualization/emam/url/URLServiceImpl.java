package de.monticore.lang.monticar.visualization.emam.url;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class URLServiceImpl implements URLService, ApplicationContribution {
    protected final Logger logger;
    protected final Set<URLContribution> contributions;
    protected final Map<String, URL> urls;

    @Inject
    public URLServiceImpl(Logger logger, Set<URLContribution> contributions) {
        this.logger = logger;
        this.contributions = contributions;
        this.urls = new HashMap<>();
    }

    @Override
    public void addURL(String id, URL url) {
        if (this.urls.containsKey(id)) this.logger.warning("A url under the id of " + id + " already exists.");
        else this.urls.put(id, url);
    }

    @Override
    public URL getURL(String id) {
        return this.urls.get(id);
    }

    @Override
    public void configure(Application application) {
        this.logger.info("Adding URLs to URLService...");

        for (URLContribution contribution : this.contributions) {
            try {
                contribution.addToRegistry(this);
            } catch(MalformedURLException exception) {
                this.logger.severe(exception.getMessage());
            }
        }
    }
}
