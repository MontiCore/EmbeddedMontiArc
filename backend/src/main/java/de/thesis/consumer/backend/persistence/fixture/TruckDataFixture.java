package de.thesis.consumer.backend.persistence.fixture;

import de.thesis.consumer.backend.domain.repository.TruckDataRepository;
import de.thesis.consumer.backend.persistence.entity.TruckData;
import lombok.AllArgsConstructor;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.time.LocalDateTime;

@Component
@AllArgsConstructor
public class TruckDataFixture implements ApplicationRunner {

	private final TruckDataRepository repository;

	@Override
	public void run(ApplicationArguments args) throws Exception {
		for (int i = 0; i < 100; i++) {
			LocalDateTime time = LocalDateTime.now();
			TruckData truckData = new TruckData();
			truckData.setDIdCIdDay("2");
			truckData.setLongitude(9 - (i * (1 / 100.0 )));
			truckData.setLatitude(52 - (i * (1 / 100.0 )));
			truckData.setGpsTime(time);
			truckData.setHeading(42);
			truckData.setSpeed(90);
			truckData.setOdometer(42000);
			truckData.setTotalFuelUsed(9);
			truckData.setValidPosition(true);
			truckData.setTimestamp(time);

			repository.save(truckData);
		}
	}
}
