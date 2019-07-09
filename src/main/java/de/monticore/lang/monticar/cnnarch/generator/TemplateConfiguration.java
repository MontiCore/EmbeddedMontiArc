/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.cnnarch.generator;

import de.se_rwth.commons.logging.Log;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import freemarker.template.TemplateExceptionHandler;

import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.util.Map;

public abstract class TemplateConfiguration {
    abstract protected String getBaseTemplatePackagePath();
    abstract public Configuration getConfiguration();

    public TemplateConfiguration() {

    }

    protected Configuration createConfiguration() {
        Configuration configuration = new Configuration(Configuration.VERSION_2_3_23);
        configuration.setClassForTemplateLoading(TemplateConfiguration.class, getBaseTemplatePackagePath());
        configuration.setDefaultEncoding("UTF-8");
        configuration.setTemplateExceptionHandler(TemplateExceptionHandler.RETHROW_HANDLER);
        return configuration;
    }

    private void quitGeneration(){
        Log.error("Code generation is aborted");
        System.exit(1);
    }

    public void processTemplate(Map<String, Object> ftlContext, String templatePath, Writer writer){
        try{
            Template template = getConfiguration().getTemplate(templatePath);
            template.process(ftlContext, writer);
        } catch (IOException e) {
            Log.error("Freemarker could not find template " + templatePath + " :\n" + e.getMessage());
            quitGeneration();
        } catch (TemplateException e){
            Log.error("An exception occured in template " + templatePath + " :\n" + e.getMessage());
            quitGeneration();
        }
    }

    public String processTemplate(Map<String, Object> ftlContext, String templatePath){
        StringWriter writer = new StringWriter();
        processTemplate(ftlContext, templatePath, writer);
        return writer.toString();
    }
}
