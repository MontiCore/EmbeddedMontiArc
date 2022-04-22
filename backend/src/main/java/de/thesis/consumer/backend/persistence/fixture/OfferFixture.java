package de.thesis.consumer.backend.persistence.fixture;

import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.persistence.entity.OfferEntity;
import lombok.AllArgsConstructor;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.util.Random;
import java.util.UUID;

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

		Offer fixedOffer = new Offer();
		fixedOffer.setId(UUID.fromString("0c2233b4-911e-451b-96e7-0ca13088e739"));
		fixedOffer.setProvider("FedEx");
		fixedOffer.setDescription("Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.");
		fixedOffer.setTitle("Fixed UUID Offer");
		fixedOffer.setPrice(randomInt(100) + randomInt(99) / 100.0);

		repo.save(fixedOffer);
		for (int i = 0; i < 20; i++) {
			Offer randomOffers = new Offer();
			randomOffers.setId(UUID.randomUUID());
			randomOffers.setProvider(provider[randomInt(provider.length)]);
			randomOffers.setDescription("Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.");
			randomOffers.setTitle(cities[randomInt(cities.length)] + " " + direction[randomInt(direction.length)]);
			randomOffers.setPrice(randomInt(100) + randomInt(99) / 100.0);

			repo.save(randomOffers);
		}
	}

	private int randomInt(int upperBound) {
		Random random = new Random();
		return random.nextInt(upperBound);
	}
}
