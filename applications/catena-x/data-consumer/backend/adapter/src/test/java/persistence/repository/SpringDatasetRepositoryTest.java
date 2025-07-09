package persistence.repository;

import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.boot.test.autoconfigure.orm.jpa.DataJpaTest;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;
import org.springframework.test.context.ContextConfiguration;
import persistence.entity.*;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;

@DataJpaTest
@ContextConfiguration(classes = {SpringDatasetRepository.class, SpringOfferRepository.class})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan("persistence.entity")
class SpringDatasetRepositoryTest {

	@Autowired
	private SpringDatasetRepository underTest;

	@Autowired
	private SpringOfferRepository offerRepository;

	@Test
	void shouldSaveDatasetAndUpdateDataRow() {
		PolicyEntity policy = new PolicyEntity();
		policy.setStartTime(LocalTime.of(8, 0));

		MetadataEntity metadata = new MetadataEntity();
		metadata.setTitle("Aachen Dataset");
		metadata.setProvider("Carrier GmbH");
		metadata.setDescription("A simple test data set...");
		metadata.setPrice(10);
		metadata.setLoggingUrl("/logging");
		metadata.setPolicy(policy);

		DataRowEntity datarow = new DataRowEntity();
		datarow.setDayID("1234");
		datarow.setLongitude(51);
		datarow.setLatitude(10);
		datarow.setGpsTime(LocalDateTime.now());
		datarow.setHeading(42);
		datarow.setSpeed(50);
		datarow.setOdometer(22000);
		datarow.setTotalFuelUsed(50);
		datarow.setTimestamp(LocalDateTime.now());

		UUID offerId = UUID.randomUUID();
		OfferEntity offer = new OfferEntity(offerId, metadata, List.of(datarow));
		offerRepository.save(offer);
		Optional<OfferEntity> savedOffer = offerRepository.findById(offerId);

		UUID datasetId = UUID.randomUUID();
		underTest.save(datasetId, LocalDateTime.now(), savedOffer.get().getMetadata().getId(), savedOffer.get().getId());

		Optional<DatasetEntity> dataset = underTest.findById(datasetId);

		assertThat(dataset.isPresent()).isTrue();
	}
}
