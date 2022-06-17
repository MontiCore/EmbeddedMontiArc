package de.thesis.consumer.config;

import entity.Dataset;
import lombok.AllArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.jdbc.core.JdbcTemplate;
import org.springframework.web.reactive.function.client.WebClient;
import persistence.mappers.JacksonDatasetMapper;
import persistence.mappers.JacksonOfferMapper;
import persistence.repository.SpringDatasetPersistencePortAdapter;
import persistence.repository.SpringDatasetRepository;
import persistence.repository.SpringOfferPersistencePortAdapter;
import persistence.repository.SpringOfferRepository;
import ports.DatasetPersistencePort;
import ports.DsEnforcementPort;
import ports.DsManagementPort;
import ports.OfferPersistencePort;
import usecases.*;

import javax.sql.DataSource;

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
										   DsManagementPort dsManagementPort) {
		return new BuyOfferUseCase(datasetPersistencePort, offerPersistencePort, dsManagementPort);
	}

	@Bean
	public GetOffersMetadataUseCase getAllOffersMetadataUseCase(OfferPersistencePort offerPersistencePort) {
		return new GetOffersMetadataUseCase(offerPersistencePort);
	}

	@Bean
	public GetAllDatasetMetadataUseCase getAllDatasetMetadataUseCase(DatasetPersistencePort datasetPersistencePort) {
		return new GetAllDatasetMetadataUseCase(datasetPersistencePort);
	}

	@Bean
	public GetDatasetUseCase getDatasetUseCase(DatasetPersistencePort datasetPersistencePort, DsEnforcementPort<Dataset> dsEnforcementPort) {
		return new GetDatasetUseCase(datasetPersistencePort, dsEnforcementPort);
	}

	@Bean
	public OfferPersistencePort offerPersistencePort(JacksonOfferMapper mapper, SpringOfferRepository repository) {
		return new SpringOfferPersistencePortAdapter(mapper, repository);
	}

	@Bean
	public DatasetPersistencePort datasetPersistencePort(JacksonDatasetMapper mapper, SpringDatasetRepository repository) {
		return new SpringDatasetPersistencePortAdapter(mapper, repository);
	}

	@Bean
	public WebClient getWebClient(@Value("${DATA_PROVIDER_URL}") String url) {
		return WebClient.create(url);
	}

	@Bean
	public JdbcTemplate jdbcTemplate(DataSource dataSource) {
		return new JdbcTemplate(dataSource);
	}

}
