package de.thesis.consumer.backend.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import de.thesis.consumer.backend.datasovereignty.pep.DatasetPolicyEnforcementPoint;
import de.thesis.consumer.backend.domain.IPolicyManagementPoint;
import de.thesis.consumer.backend.domain.repository.DatasetRepository;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.domain.repository.PolicyRepository;
import de.thesis.consumer.backend.domain.service.*;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.reactive.function.client.WebClient;

@Configuration
public class CompositionConfig {

	@Bean
	public OfferService getOfferService(
			OfferRepository offerRepository,
			PolicyRepository policyRepository,
			DatasetRepository datasetRepository,
			IPolicyService policyService,
			IPolicyManagementPoint pmp,
			ObjectMapper mapper) {
		return new OfferService(offerRepository, policyRepository, datasetRepository, policyService, pmp, mapper);
	}

	@Bean
	public DatasetService getDatasetService(DatasetRepository datasetRepository, DatasetPolicyEnforcementPoint pep) {
		return new DatasetService(datasetRepository, pep);
	}

	@Bean
	public WebClient getWebClient(@Value("${DATA_PROVIDER_URL}") String url) {
		return WebClient.create(url);
	}

	@Bean
	public ProviderClient getProviderClient(IHttpClient client) {
		return new ProviderClient(client);
	}
}
