package de.thesis.provider.backend.dto;

import lombok.Getter;
import lombok.Setter;

import java.time.LocalDate;
import java.util.UUID;

@Getter
@Setter
public class CreatePolicyDto {
	private UUID id;
	private String event;
	private BusinessHours businessHours;
	private int maxUsages;
	private LocalDate expiresOn;
	private boolean localLogging;
	private boolean remoteLogging;
}
