package de.thesis.provider.backend.policy;

import lombok.Value;

import java.time.LocalTime;

@Value
public class BusinessHours {
	LocalTime start;
	LocalTime end;
}
