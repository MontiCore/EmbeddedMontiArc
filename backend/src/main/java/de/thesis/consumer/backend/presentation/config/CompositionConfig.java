package de.thesis.consumer.backend.presentation.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.domain.repository.TruckDataRepository;
import de.thesis.consumer.backend.domain.service.DatasetService;
import de.thesis.consumer.backend.domain.service.OfferService;
import de.thesis.consumer.backend.domain.service.PolicyServiceMock;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class CompositionConfig {

	@Bean
	public OfferService getOfferService(OfferRepository offerRepository, DatasetRepository datasetRepository, TruckDataRepository truckDataRepository, ObjectMapper mapper) {
		return new OfferService(offerRepository, datasetRepository, truckDataRepository, new PolicyServiceMock(), mapper);
	}

	@Bean
	public DatasetService getDatasetService(DatasetRepository datasetRepository) {
		return new DatasetService(datasetRepository);
	}
}
