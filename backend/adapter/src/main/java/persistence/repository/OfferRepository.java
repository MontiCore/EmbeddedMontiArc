package persistence.repository;

import entity.Offer;
import lombok.AllArgsConstructor;
import org.springframework.stereotype.Repository;
import org.springframework.web.bind.annotation.RestController;
import port.OfferPersistencePort;

import java.util.UUID;

@Repository
@AllArgsConstructor
public class OfferRepository implements OfferPersistencePort {

	private final SpringOfferRepository repository;

	@Override
	public void save(Offer offer) {
		System.err.println("i was called with title: ");
		System.err.println(offer.getTitle());
	}

	@Override
	public Iterable<Offer> findAll() {
		return null;
	}

	@Override
	public Offer findBy(UUID offerId) {
		return null;
	}
}
