# pragma once

struct MechanicalParamerters {
	float MassPendulum = 0.071f;
	float MassCart = 0.288f;
	float LengthPendulum = 0.685f - 0.246f;
	float InertiaPendulum = 0.0000005f;
	float b_pendulum = 0.00001; // Pendulum viscous damping
	float b_cart = 0.1f;  // Cart viscous damping
}