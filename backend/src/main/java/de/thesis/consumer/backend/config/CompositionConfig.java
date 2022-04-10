package de.thesis.consumer.backend.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.DatasetRepository;
import de.thesis.consumer.backend.domain.OfferRepository;
import de.thesis.consumer.backend.domain.OfferService;
import de.thesis.consumer.backend.domain.PolicyServiceMock;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class CompositionConfig {

	@Bean
	public OfferService getDatasetService(OfferRepository offerRepository, DatasetRepository datasetRepository, ObjectMapper mapper) {
		return new OfferService(offerRepository, datasetRepository, new PolicyServiceMock(), mapper);
	}
}
