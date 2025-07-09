package persistence.repository;

import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.autoconfigure.domain.EntityScan;
import org.springframework.boot.test.autoconfigure.orm.jpa.DataJpaTest;
import org.springframework.data.jpa.repository.config.EnableJpaRepositories;
import org.springframework.test.context.ContextConfiguration;
import persistence.entity.DataRowEntity;
import persistence.entity.MetadataEntity;
import persistence.entity.OfferEntity;
import persistence.entity.PolicyEntity;

import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;
import java.util.UUID;

import static org.assertj.core.api.Assertions.assertThat;

@DataJpaTest
@ContextConfiguration(classes = {SpringOfferRepository.class})
@EnableJpaRepositories(basePackages = {"persistence.repository"})
@EntityScan("persistence.entity")
class SpringOfferRepositoryTest {

	@Autowired
	private SpringOfferRepository underTest;

	@Test
	void shouldDeleteOffer() {
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

		UUID uuid = UUID.fromString("d1040e67-e982-4b4d-beab-2fe09c7b8be8");
		OfferEntity offer = new OfferEntity(uuid, metadata, List.of(datarow));

		underTest.save(offer);

		underTest.deleteById(uuid);

		int count = 0;
		for (OfferEntity ignored : underTest.findAll()) {
			count++;
		}

		assertThat(count).isEqualTo(0);
	}
}
