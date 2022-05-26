package de.thesis.consumer.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import datasovereingty.PolicyManagementPointAdapter;
import de.fraunhofer.iese.mydata.IMyDataEnvironment;
import entity.Policy;
import entity.Timer;
import lombok.AllArgsConstructor;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import persistence.mappers.JacksonDatasetMapper;
import persistence.mappers.JacksonOfferMapper;
import persistence.repository.SpringDatasetRepository;
import persistence.repository.SpringDatasetRepositoryAdapter;
import persistence.repository.SpringOfferRepository;
import persistence.repository.SpringOfferRepositoryAdapter;
import ports.DatasetPersistencePort;
import ports.OfferPersistencePort;
import ports.PolicyManagementPort;
import usecases.AddOfferUseCase;
import usecases.BuyOfferUseCase;

@Configuration
@AllArgsConstructor
public class CompositionConfig {

	private final IMyDataEnvironment myDataEnvironment;

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
	public OfferPersistencePort offerPersistencePort(JacksonOfferMapper mapper, SpringOfferRepository repository) {
		return new SpringOfferRepositoryAdapter(mapper, repository);
	}

	@Bean
	public DatasetPersistencePort datasetPersistencePort(JacksonDatasetMapper mapper, SpringDatasetRepository repository) {
		return new SpringDatasetRepositoryAdapter(mapper, repository);
	}

	@Bean
	public JacksonDatasetMapper jacksonDatasetMapper(ObjectMapper mapper) {
		return new JacksonDatasetMapper(mapper);
	}

	@Bean
	public JacksonOfferMapper jacksonOfferMapper(ObjectMapper mapper) {
		return new JacksonOfferMapper(mapper);
	}

	@Bean
	public PolicyManagementPort policyManagementPort() {
		return new PolicyManagementPointAdapter(myDataEnvironment);
	}
}
