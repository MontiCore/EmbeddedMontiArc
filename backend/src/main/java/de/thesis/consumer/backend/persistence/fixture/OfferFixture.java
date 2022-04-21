package de.thesis.consumer.backend.persistence.fixture;

import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import lombok.AllArgsConstructor;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.util.Random;

@Component
@AllArgsConstructor
public class OfferFixture implements ApplicationRunner {

	private final OfferRepository repo;

	@Override
	public void run(ApplicationArguments args) {
		String[] provider = {"Dachser", "FedEx", "DBCargo", "UPS", "DHL", "DPD"};
		String[] cities = {"Berlin", "Hamburg", "Hannover", "Wolfsburg", "Düsseldorf", "Köln", "Mannheim", "Aachen",
				"Friedrichshafen", "Dortmund", "München", "Augsburg", "Stuttgart", "Leipzig"};
		String[] direction = {"West", "Nord", "Ost", "Süd"};
		for (int i = 0; i < 20; i++) {
			Offer offer = new Offer();
			offer.setProvider(provider[randomInt(provider.length)]);
			offer.setDescription("Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.");
			offer.setTitle(cities[randomInt(cities.length)] + " " + direction[randomInt(direction.length)]);
			offer.setPrice(randomInt(100) + randomInt(99) / 100.0);

			repo.save(offer);
		}
	}

	private int randomInt(int upperBound) {
		Random random = new Random();
		return random.nextInt(upperBound);
	}
}
