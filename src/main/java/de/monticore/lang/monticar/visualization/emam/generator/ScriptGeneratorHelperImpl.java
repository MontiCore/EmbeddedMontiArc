/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.generator;

import com.google.common.base.Preconditions;
import com.google.inject.Singleton;

@Singleton
public class ScriptGeneratorHelperImpl implements ScriptGeneratorHelper {
    protected String packige;
    protected String name;
    protected String body;

    @Override
    public void setPackage(String packige) {
        Preconditions.checkNotNull(packige);

        this.packige = packige;
    }

    @Override
    public String getPackage() {
        return this.packige;
    }

    @Override
    public void setName(String name) {
        Preconditions.checkNotNull(name);

        this.name = name;
    }

    @Override
    public String getName() {
        return this.name;
    }

    @Override
    public void setBody(String body) {
        Preconditions.checkNotNull(body);

        this.body = body;
    }

    @Override
    public String getBody() {
        return this.body;
    }
}
