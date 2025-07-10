package de.thesis.provider.backend.controllers;

import de.thesis.provider.backend.dto.CreateOfferCommand;
import de.thesis.provider.backend.dto.Metadata;
import de.thesis.provider.backend.services.OfferService;
import lombok.AllArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.util.Map;
import java.util.UUID;

@RestController
@RequestMapping("/offers")
@AllArgsConstructor
@CrossOrigin("*")
public class OfferController {

	private final OfferService service;

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public void createOffer(@RequestBody CreateOfferCommand command,
							@Value("${provider.name}") String providerName,
							@Value("${provider.url}") String providerUrl) {
		command.getMetadata().setProvider(providerName);
		command.getMetadata().setLoggingUrl(providerUrl);
		service.addOffer(command);
	}

	@GetMapping
	@ResponseStatus(HttpStatus.OK)
	public Map<UUID, Metadata> getOffers() {
		return service.getOffers();
	}

	@DeleteMapping("/{offerId}")
	@ResponseStatus(HttpStatus.OK)
	public void deleteOffer(@PathVariable UUID offerId) {
		service.deleteOffer(offerId);
	}
}
