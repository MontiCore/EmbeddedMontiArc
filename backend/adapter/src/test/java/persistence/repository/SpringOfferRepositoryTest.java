package persistence.repository;

import org.junit.jupiter.api.AfterEach;
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
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;

@DataJpaTest
@ContextConfiguration(classes = {SpringOfferRepository.class, SpringDatasetRepository.class})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan("persistence.entity")
class SpringOfferRepositoryTest {

	@Autowired
	private SpringOfferRepository underTest;

	@Autowired
	private SpringDatasetRepository datasetRepository;

	@Test
	void shouldFindOneBoughtOfferOutOfTwo() {
		// given
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

		OfferEntity offer = new OfferEntity(UUID.randomUUID(), metadata, List.of(datarow));
		OfferEntity boughOffer = new OfferEntity(UUID.randomUUID(), metadata, List.of(datarow));

		underTest.save(offer);
		underTest.save(boughOffer);
		DatasetEntity dataset = new DatasetEntity();
		dataset.setId(UUID.randomUUID());
		dataset.setOffer(boughOffer);
		dataset.setBoughtAt(LocalDateTime.now());
		dataset.setMetadata(metadata);
		dataset.setData(List.of(datarow));
		datasetRepository.save(dataset);

		// when
		int count = 0;
		for (OfferEntity ignored : underTest.findAllBoughtOffers()) {
			count++;
		}

		// then
		assertThat(count).isEqualTo(1);
	}
}
