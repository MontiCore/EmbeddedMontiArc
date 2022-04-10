package de.thesis.consumer.backend.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.*;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class CompositionConfig {

	@Bean
	public OfferService getOfferService(OfferRepository offerRepository, DatasetRepository datasetRepository, ObjectMapper mapper) {
		return new OfferService(offerRepository, datasetRepository, new PolicyServiceMock(), mapper);
	}

	@Bean
	public DatasetService getDatasetService(DatasetRepository datasetRepository) {
		return new DatasetService(datasetRepository);
	}
}
