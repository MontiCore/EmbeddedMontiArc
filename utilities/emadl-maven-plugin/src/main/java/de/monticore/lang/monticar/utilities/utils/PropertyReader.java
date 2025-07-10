package de.monticore.lang.monticar.utilities.utils;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class PropertyReader {
    Properties p = new Properties();

    public PropertyReader() {
        InputStream is = this.getClass().getClassLoader().getResourceAsStream("app.properties");
        try {
            this.p.load(is);
        } catch (IOException e){
            e.printStackTrace();
        }
    }

    public String getProperty(String key){
        return this.p.getProperty(key);
    }
}
