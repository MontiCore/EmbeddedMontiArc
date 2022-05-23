package presentation.controller;

import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RequestMapping("/offers")
@RestController
public class OfferController {

	@GetMapping
	public String test() {
		return "ich bin ein test";
	}
}
