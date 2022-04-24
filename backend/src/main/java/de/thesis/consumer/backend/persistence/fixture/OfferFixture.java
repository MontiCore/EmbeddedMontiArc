package de.thesis.consumer.backend.persistence.fixture;

import de.thesis.consumer.backend.domain.model.Offer;
import de.thesis.consumer.backend.domain.model.Policy;
import de.thesis.consumer.backend.domain.repository.OfferRepository;
import de.thesis.consumer.backend.domain.repository.PolicyRepository;
import lombok.AllArgsConstructor;
import lombok.SneakyThrows;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.util.Random;
import java.util.UUID;

@Component
@AllArgsConstructor
public class OfferFixture implements ApplicationRunner {

	private final OfferRepository repo;
	private final PolicyRepository policyRepository;

	@SneakyThrows
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
		Policy policy = new Policy("urn:policy:rwth-student-solution:0c2233b4-911e-451b-96e7-0ca13088e739", "<policy id='urn:policy:rwth-student-solution:0c2233b4-911e-451b-96e7-0ca13088e739' xmlns='http://www.mydata-control.de/4.0/mydataLanguage' xmlns:tns='http://www.mydata-control.de/4.0/mydataLanguage' xmlns:parameter='http://www.mydata-control.de/4.0/parameter' xmlns:pip='http://www.mydata-control.de/4.0/pip' xmlns:function='http://www.mydata-control.de/4.0/function' xmlns:event='http://www.mydata-control.de/4.0/event' xmlns:constant='http://www.mydata-control.de/4.0/constant' xmlns:variable='http://www.mydata-control.de/4.0/variable' xmlns:variableDeclaration='http://www.mydata-control.de/4.0/variableDeclaration' xmlns:valueChanged='http://www.mydata-control.de/4.0/valueChanged' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xmlns:date='http://www.mydata-control.de/4.0/date' xmlns:time='http://www.mydata-control.de/4.0/time' xmlns:day='http://www.mydata-control.de/4.0/day'>\n" +
				"    <mechanism event='urn:action:rwth-student-solution:dataset-access'>\n" +
				"        <if>\n" +
				"            <equals>\n" +
				"        			 <constant:string value='0c2233b4-911e-451b-96e7-0ca13088e739'/>\n" +
				"        		  	 <event:string eventParameter='dataset' default='' jsonPathQuery='$.id'/>\n" +
				"      		 </equals>\n" +
				"            <then>\n" +
				"                <modify eventParameter='dataset' method='replace' jsonPathQuery='$.description'>\n" +
				"                    <parameter:string name='replaceWith' value='testwert'/>\n" +
				"                </modify>\n" +
				"            </then>\n" +
				"        </if>\n" +
				"    </mechanism>\n" +
				"</policy>");

		fixedOffer.setPolicy(policy);
		policyRepository.save(policy);

		repo.save(fixedOffer);
		for (int i = 0; i < 20; i++) {
			Offer randomOffers = new Offer();
			randomOffers.setId(UUID.randomUUID());
			randomOffers.setProvider(provider[randomInt(provider.length)]);
			randomOffers.setDescription("Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet.");
			randomOffers.setTitle(cities[randomInt(cities.length)] + " " + direction[randomInt(direction.length)]);
			randomOffers.setPrice(randomInt(100) + randomInt(99) / 100.0);

			Policy randomPolicy = new Policy(UUID.randomUUID().toString(), "test");
			randomOffers.setPolicy(randomPolicy);
			repo.save(randomOffers);
		}
	}

	private int randomInt(int upperBound) {
		Random random = new Random();
		return random.nextInt(upperBound);
	}
}
