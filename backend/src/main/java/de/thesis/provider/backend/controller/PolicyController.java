package de.thesis.provider.backend.controller;

import de.thesis.provider.backend.csv.CsvReader;
import de.thesis.provider.backend.dto.CreatePolicyDto;
import de.thesis.provider.backend.policy.PolicyService;
import freemarker.template.TemplateException;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;

@RestController
@RequestMapping("/policy")
@AllArgsConstructor
@CrossOrigin(origins = "*")
public class PolicyController {

	private final PolicyService policyService;

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public void getPolicy(@RequestBody CreatePolicyDto policyRequest) throws TemplateException, IOException {
		System.err.println(policyService.getPolicy(policyRequest));
	}

	@GetMapping
	@ResponseStatus(HttpStatus.OK)
	public void readCSV() throws Exception {
		CsvReader reader = new CsvReader();
		reader.getCsvData();
	}
}
