package de.thesis.consumer.backend.config;

import de.thesis.consumer.backend.domain.OfferRepository;
import de.thesis.consumer.backend.domain.OfferService;
import de.thesis.consumer.backend.domain.PolicyServiceMock;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class CompositionConfig {

	@Bean
	public OfferService getDatasetService(OfferRepository repository) {
		return new OfferService(repository, new PolicyServiceMock());
	}
}
