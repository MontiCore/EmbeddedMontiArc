package de.thesis.provider.backend.policy;

import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.io.StringWriter;
import java.util.HashMap;
import java.util.Map;

@Service
public class PolicyService {

	private final Configuration config = new Configuration(Configuration.VERSION_2_3_23);

	public PolicyService() {
		config.setClassForTemplateLoading(PolicyService.class, "/templates");
		config.setDefaultEncoding("UTF-8");
	}

	public String getPolicy(PolicyRequest policyRequest) throws IOException, TemplateException {
		Map<String, Object> templateData = new HashMap<>();
		templateData.put("id", policyRequest.getId());
		templateData.put("event", policyRequest.getEvent());
		templateData.put("maxUsages", policyRequest.getMaxUsages());
		templateData.put("startTime", policyRequest.getStartTime());
		templateData.put("endTime", policyRequest.getEndTime());
		templateData.put("localLogging", policyRequest.isLocalLogging());
		templateData.put("remoteLogging", policyRequest.isRemoteLogging());

		Template template = config.getTemplate("policy.ftl");
		StringWriter stringWriter = new StringWriter();
		template.process(templateData, stringWriter);
		return stringWriter.toString();
	}
}
