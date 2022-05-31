package de.thesis.consumer.config;

import entity.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.reactive.function.client.WebClient;
import persistence.mappers.JacksonDatasetMapper;
import persistence.mappers.JacksonOfferMapper;
import persistence.repository.SpringDatasetRepository;
import persistence.repository.SpringDatasetRepositoryAdapter;
import persistence.repository.SpringOfferRepository;
import persistence.repository.SpringOfferPersistencePortAdapter;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;
import ports.PolicyEnforcementPort;
import ports.PolicyManagementPort;
import usecases.*;

@Configuration
@AllArgsConstructor
public class CompositionConfig {

	@Bean
	public AddOfferUseCase addOfferUseCase(OfferPersistencePort offerPersistencePort) {
		return new AddOfferUseCase(offerPersistencePort);
	}

	@Bean
	public BuyOfferUseCase buyOfferUseCase(DatasetPersistencePort datasetPersistencePort,
										   OfferPersistencePort offerPersistencePort,
										   PolicyManagementPort policyManagementPort) {
		return new BuyOfferUseCase(datasetPersistencePort, offerPersistencePort, policyManagementPort);
	}

	@Bean
	public GetAllOffersMetadataUseCase getAllOffersMetadataUseCase(OfferPersistencePort offerPersistencePort) {
		return new GetAllOffersMetadataUseCase(offerPersistencePort);
	}

	@Bean
	public GetAllDatasetMetadataUseCase getAllDatasetMetadataUseCase(DatasetPersistencePort datasetPersistencePort) {
		return new GetAllDatasetMetadataUseCase(datasetPersistencePort);
	}

	@Bean
	public GetDatasetUseCase getDatasetUseCase(DatasetPersistencePort datasetPersistencePort, PolicyEnforcementPort<Dataset> policyEnforcementPort) {
		return new GetDatasetUseCase(datasetPersistencePort, policyEnforcementPort);
	}

	@Bean
	public OfferPersistencePort offerPersistencePort(JacksonOfferMapper mapper, SpringOfferRepository repository) {
		return new SpringOfferPersistencePortAdapter(mapper, repository);
	}

	@Bean
	public DatasetPersistencePort datasetPersistencePort(JacksonDatasetMapper mapper, SpringDatasetRepository repository) {
		return new SpringDatasetRepositoryAdapter(mapper, repository);
	}

	@Bean
	public WebClient getWebClient(@Value("${DATA_PROVIDER_URL}") String url) {
		return WebClient.create(url);
	}

}
