package de.thesis.consumer.backend.config;

import de.thesis.consumer.backend.domain.DatasetRepository;
import de.thesis.consumer.backend.domain.DatasetService;
import de.thesis.consumer.backend.domain.PolicyServiceMock;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class CompositionConfig {

	@Bean
	public DatasetService getDatasetService(DatasetRepository repository) {
		return new DatasetService(repository, new PolicyServiceMock());
	}
}
