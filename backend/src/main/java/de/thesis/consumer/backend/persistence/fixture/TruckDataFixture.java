package de.thesis.consumer.backend.persistence.fixture;

import de.thesis.consumer.backend.domain.model.DataRow;
import de.thesis.consumer.backend.domain.repository.DataRowRepository;
import lombok.AllArgsConstructor;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.time.LocalDateTime;

@Component
@AllArgsConstructor
public class TruckDataFixture implements ApplicationRunner {

	private final DataRowRepository repository;

	@Override
	public void run(ApplicationArguments args) throws Exception {
		for (int i = 0; i < 100; i++) {
			LocalDateTime time = LocalDateTime.now();
			DataRow row = new DataRow();
			row.setDIdCIdDay("2");
			row.setLongitude(9 - (i * (1 / 100.0)));
			row.setLatitude(52 - (i * (1 / 100.0)));
			row.setGpsTime(time);
			row.setHeading(42);
			row.setSpeed(90);
			row.setOdometer(42000);
			row.setTotalFuelUsed(9);
			row.setValidPosition(true);
			row.setTimestamp(time);

			repository.save(row);
		}
	}
}
