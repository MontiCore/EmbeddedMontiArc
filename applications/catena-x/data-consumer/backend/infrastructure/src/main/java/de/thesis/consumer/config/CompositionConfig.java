package de.thesis.consumer.config;

import entity.Dataset;
import entity.Offer;
import insurance.InsuranceFeeCalculatorPortAdapter;
import insurance.RandomFeeCalculator;
import lombok.AllArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.reactive.function.client.WebClient;
import persistence.entity.DatasetEntity;
import persistence.entity.OfferEntity;
import persistence.mappers.Mapper;
import persistence.repository.SpringDatasetPersistencePortAdapter;
import persistence.repository.SpringDatasetRepository;
import persistence.repository.SpringOfferPersistencePortAdapter;
import persistence.repository.SpringOfferRepository;
import ports.*;
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
										   PolicyDeploymentPort policyDeploymentPort) {
		return new BuyOfferUseCase(datasetPersistencePort, offerPersistencePort, policyDeploymentPort);
	}

	@Bean
	public DeleteDatasetUseCase getDeleteDatasetUseCase(DatasetPersistencePort datasetPersistencePort) {
		return new DeleteDatasetUseCase(datasetPersistencePort);
	}

	@Bean
	public DeleteOfferUseCase getDeleteOfferUseCase(DatasetPersistencePort datasetPersistencePort, OfferPersistencePort offerPersistencePort) {
		return new DeleteOfferUseCase(datasetPersistencePort, offerPersistencePort);
	}

	@Bean
	public GetAllDatasetMetadataUseCase getAllDatasetMetadataUseCase(DatasetPersistencePort datasetPersistencePort) {
		return new GetAllDatasetMetadataUseCase(datasetPersistencePort);
	}

	@Bean
	public GetDatasetViewUseCase getDatasetViewUseCase(DatasetPersistencePort datasetPersistencePort,
													   InsuranceFeeCalculatorPort insuranceFeeCalculatorPort,
													   PolicyEnforcementPort<Dataset> policyEnforcementPort) {
		return new GetDatasetViewUseCase(datasetPersistencePort, insuranceFeeCalculatorPort, policyEnforcementPort);
	}

	@Bean
	public GetAllOffersMetadataUseCase getAllOffersMetadataUseCase(OfferPersistencePort offerPersistencePort) {
		return new GetAllOffersMetadataUseCase(offerPersistencePort);
	}

	@Bean
	public RemoteLoggingUseCase getRemoteLoggingUseCase(ProviderCommunicationPort communicationPort) {
		return new RemoteLoggingUseCase(communicationPort);
	}

	@Bean
	public RemoveExpiredDatasetsUseCase getRemoveExpiredDatasetsUseCase(DatasetPersistencePort datasetPersistencePort) {
		return new RemoveExpiredDatasetsUseCase(datasetPersistencePort);
	}

	@Bean
	public OfferPersistencePort offerPersistencePort(Mapper<Offer, OfferEntity> mapper, SpringOfferRepository repository) {
		return new SpringOfferPersistencePortAdapter(mapper, repository);
	}

	@Bean
	public DatasetPersistencePort datasetPersistencePort(Mapper<Dataset, DatasetEntity> mapper, SpringDatasetRepository repository) {
		return new SpringDatasetPersistencePortAdapter(mapper, repository);
	}

	@Bean
	public InsuranceFeeCalculatorPort getInsuranceFeeCalculatorPort(RandomFeeCalculator calculator) {
		return new InsuranceFeeCalculatorPortAdapter(calculator);
	}

	@Bean
	public WebClient getWebClient(@Value("${DATA_PROVIDER_URL}") String url) {
		return WebClient.create(url);
	}


}
