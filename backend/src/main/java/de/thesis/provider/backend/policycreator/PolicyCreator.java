package de.thesis.provider.backend.policycreator;

import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;

import java.io.IOException;
import java.io.StringWriter;
import java.time.LocalTime;
import java.util.HashMap;
import java.util.Map;

public class PolicyCreator {
	public static void createPolicy() throws IOException, TemplateException {
		Map<String, Object> templateData = new HashMap<>();
		templateData.put("id", "4916dfec-6d4d-4ef1-a550-a67000b8f9ba");
		templateData.put("event", "dataset-access");
		templateData.put("timeRule", new TimeRule(LocalTime.of(8, 0), null));
		templateData.put("localLogging", true);
		templateData.put("remoteLogging", true);
		Configuration conf = new Configuration(Configuration.VERSION_2_3_23);
		conf.setClassForTemplateLoading(PolicyCreator.class, "/templates");
		conf.setDefaultEncoding("UTF-8");

		Template template = conf.getTemplate("policy.ftl");

		StringWriter stringWriter = new StringWriter();
		template.process(templateData, stringWriter);

		System.err.println(stringWriter);
	}
}
