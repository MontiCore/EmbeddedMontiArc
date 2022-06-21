package datasovereignty;

import entity.Policy;
import freemarker.template.Configuration;
import freemarker.template.Template;
import freemarker.template.TemplateException;
import lombok.AllArgsConstructor;
import lombok.Getter;
import org.springframework.stereotype.Service;

import java.io.IOException;
import java.io.StringWriter;
import java.time.LocalTime;
import java.util.HashMap;
import java.util.Map;

@Service
public class MydataPolicyFactory {

	private final String EVENT_NAME = "dataset-access";
	private final Configuration config = new Configuration(Configuration.VERSION_2_3_23);

	public MydataPolicyFactory() {
		config.setClassForTemplateLoading(MydataPolicyFactory.class, "/templates");
		config.setDefaultEncoding("UTF-8");
	}

	public String getMydataPolicy(Policy policy) throws IOException, TemplateException {
		Map<String, Object> templateData = new HashMap<>();
		templateData.put("id", policy.getTargetId());
		templateData.put("event", EVENT_NAME);
		templateData.put("maxUsages", policy.getMaxUsages());
		templateData.put("businessHours", new BusinessHours(policy.getStartTime(), policy.getEndTime()));
		templateData.put("localLogging", policy.isLocalLogging());
		templateData.put("remoteLogging", policy.isRemoteLogging());

		Template template = config.getTemplate("policy.ftl");
		StringWriter stringWriter = new StringWriter();
		template.process(templateData, stringWriter);

		return stringWriter.toString();
	}

	@Getter
	@AllArgsConstructor
	public static class BusinessHours {
		private LocalTime start;
		private LocalTime end;
	}
}
