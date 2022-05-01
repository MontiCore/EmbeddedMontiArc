package de.thesis.provider.backend.policy;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDate;
import java.util.UUID;

@Getter
@Setter
public class PolicyRequest {
	private UUID id;
	private String event;
	private TimeRule businessHours;
	private Integer maxUsages;
	private LocalDate expiresOn;
	private boolean localLogging;
	private boolean remoteLogging;
}
