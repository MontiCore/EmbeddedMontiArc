package de.thesis.consumer.backend.domain.model;

import lombok.Getter;
import lombok.Setter;

import java.util.UUID;

@Getter
@Setter
public class Policy {
	private UUID id;
	private String rawValue;
}
