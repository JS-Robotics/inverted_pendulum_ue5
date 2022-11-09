# pragma once

struct MechanicalParamerters {
	float m_pendulum = 0.071f;
	float m_cart = 0.288f;
	float l_pendulum = 0.685f - 0.246f;
	float I_pendulum = 0.0000005f;
	float b_pendulum = 0.00001; // Pendulum viscous damping
	float b_cart = 0.1f;  // Cart viscous damping
}