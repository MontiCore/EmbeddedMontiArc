package de.thesis.provider.backend;

import de.thesis.provider.backend.policy.PolicyRequest;
import de.thesis.provider.backend.policy.PolicyService;
import freemarker.template.TemplateException;
import lombok.AllArgsConstructor;
import org.springframework.http.HttpStatus;
import org.springframework.web.bind.annotation.*;

import java.io.IOException;

@RestController
@RequestMapping("/policy")
@AllArgsConstructor
public class PolicyController {

	private final PolicyService policyService;

	@PostMapping
	@ResponseStatus(HttpStatus.OK)
	public void getPolicy(@RequestBody PolicyRequest policyRequest) throws TemplateException, IOException {
		System.err.println(policyService.getPolicy(policyRequest));
	}
}
