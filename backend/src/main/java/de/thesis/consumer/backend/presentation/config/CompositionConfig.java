package de.thesis.consumer.backend.presentation.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.datasovereignty.pep.DatasetPolicyEnforcementPoint;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.repository.DataRowRepository;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.domain.repository.PolicyRepository;
import de.thesis.consumer.backend.domain.service.DatasetService;
import de.thesis.consumer.backend.domain.service.IPolicyService;
import de.thesis.consumer.backend.domain.service.OfferService;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

@Configuration
public class CompositionConfig {

	@Bean
	public OfferService getOfferService(
			OfferRepository offerRepository,
			PolicyRepository policyRepository,
			DatasetRepository datasetRepository,
			DataRowRepository truckDataRepository,
			IPolicyService policyService,
			IPolicyManagementPoint pmp,
			ObjectMapper mapper) {
		return new OfferService(offerRepository, policyRepository, datasetRepository, truckDataRepository, policyService, pmp, mapper);
	}

	@Bean
	public DatasetService getDatasetService(DatasetRepository datasetRepository, DatasetPolicyEnforcementPoint pep) {
		return new DatasetService(datasetRepository, pep);
	}
}
