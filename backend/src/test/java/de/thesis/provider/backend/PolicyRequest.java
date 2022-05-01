package de.thesis.provider.backend;

import lombok.Getter;
import lombok.Setter;
import org.springframework.web.bind.annotation.GetMapping;

@Getter
@Setter
public class PolicyRequest {
	int maxUsages;
}
