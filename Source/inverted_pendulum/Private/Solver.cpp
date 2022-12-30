// Fill out your copyright notice in the Description page of Project Settings.


#include "Solver.h"
#include <thread>

FSolver::FSolver(UWorld* UWorldPtr): WorldPtr(UWorldPtr)
{
	Thread = FRunnableThread::Create(this, TEXT("Solver Thread"));
	Thread->SetThreadPriority(EThreadPriority::TPri_Highest);
	const float WorldTime = WorldPtr->GetWorld()->GetRealTimeSeconds();
	UE_LOG(LogTemp, Warning, TEXT("Solver Thread GetWorld.GetTime() =  %f"), WorldTime);
	ErrorIntegral = 0;
}

FSolver::~FSolver()
{
	if (Thread != nullptr)
	{
		Thread->Kill(true);
		delete Thread;
	}
}

bool FSolver::Init()
{
	bStopThread = false;
	CartPosition = 0.0f;
	PoleRotation = 0.0f;
	SetPoint = 0.0f;
	SetPointIn = SetPoint;
	return true;
}

uint32 FSolver::Run()
{
	// https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling
	float DeltaTime = 0.005;
	//DeltaTime = 1.0f/300.0f;
	float StepTime = 0.0f;

	float x = 0.0f;
	float x_d = 0.0f;
	float x_dd = 0.0f;


	float w = PI; //2.0f; //3.1415f;
	float w_d = 0.0f;
	float w_dd = 0.0f;

	float b_c = 0.1f; //1.15f;
	float b_p = 0.0001f; //0.35f; //1.17f;
	// float b_p = 0.0075f;

	float F_m = 0.0f;

	float x_ref = 0.f;
	float x_d_ref = 0.f;
	float w_ref = PI * 0.0f;
	float w_d_ref = 0.f;
	// float k_gains[4] = {-31.6227766, -51.12423315, 584.51134517, 110.81444504}; // 10kg
	float k_gains[4] = {-31.6227766, -75.24925849, 863.38900959, 86.05398167};


	float m_p = 0.071f;
	float m_c = 0.288f;
	float L_p = (0.685f - 0.246f);
	float I_p = 0.0000005f; //0.006f;
	float g = 9.81f;
	float u = 0;
	bool SwingUp = true;
	bool UseLQR = false;
	float e = 0.35f;
	float rail_ends = 0.415f;

	std::chrono::steady_clock::time_point StepStartTime;
	std::chrono::steady_clock::time_point StepEndTime;
	std::chrono::steady_clock::time_point Timer = std::chrono::steady_clock::now();
	
	while (!bStopThread)
	{
		StepStartTime = std::chrono::steady_clock::now();

		F_m = u;

		// if(F_m == 0.f)  //TODO Figure out how to model the holding of a stopped motor.
		// {
		// 	F_m = -(m_c + m_p) * (x_d / 0.1f); // Stopping the cart Almost instantanious when motor is not active
		// }

		x_dd = (F_m - b_c * x_d + m_p * L_p * w_dd * cos(w) - m_p * L_p * w_d * w_d * sin(w)) / (m_p + m_c);
		x_d = x_dd * DeltaTime + x_d;
		x = x_d * DeltaTime + x;

		// Collision handling -- START
		if (x >= rail_ends)
		{
			x_dd = 0.f;
			x_d = -e * x_d;
			x = rail_ends;
		}
		if (x <= -rail_ends)
		{
			x_dd = 0.f;
			x_d = -e * x_d;
			x = -rail_ends;
		} // Collision handling -- End

		w_dd = (m_p * L_p * g * sin(w) + m_p * L_p * x_dd * cos(w) - b_p * w_d) / (I_p + m_p * L_p * L_p);
		w_d = w_dd * DeltaTime + w_d;
		w = w_d * DeltaTime + w;

		if (SwingUp)
		{
			u = SwingUpControl(w, w_d, x, x_d);

			if (w < 0.0 + 0.15 || w > 3.1415 * 2 - 0.15)
			{
				SwingUp = false;
				UseLQR = true;
				if (w > 3.1415 * 2 - 0.15)
				{
					w_ref = PI * 2;
				}
			}
		}

		if (UseLQR)
		{
			const float temp_x = x - x_ref;
			const float temp_xd = x_d - x_d_ref;
			const float temp_w = w - w_ref;
			const float temp_wd = w_d - w_d_ref;
			u = -k_gains[0] * temp_x - k_gains[1] * temp_xd - k_gains[2] * temp_w - k_gains[3] * temp_wd;
		}

		if (u > 100.f)
		{
			u = 100.f;
		}

		UE4_Mutex.Lock();
		CartPosition = x * 100.0f; // Converting from [m] to [cm]
		// PoleRotation = w * 180.0f / 3.14f - 180.f; // Converting from [rad] to [deg], and rotating to UE axis
		PoleRotation = w; // Converting from [rad] to [deg], and rotating to UE axis
		CartForce = F_m;
		SetPointIn = SetPoint * 100.0f; // Variable to make it thread safe
		UE4_Mutex.Unlock();

		StepEndTime = std::chrono::steady_clock::now();
		StepTime = std::chrono::duration<float>(StepEndTime - StepStartTime).count();

		if (StepTime < DeltaTime)
		{
			std::this_thread::sleep_for(std::chrono::duration<double>(DeltaTime - StepTime));
		}

		ElapsedTime = std::chrono::duration<float>(std::chrono::steady_clock::now() - Timer).count();
	}
	return 0;
}


float FSolver::SwingUpControl(float Theta, float ThetaDot, float Position, float Velocity)
{
	float m_p = 0.071f;
	float m_c = 0.288f;
	float L_p = (0.685f - 0.246f);
	float I_p = 0.0000005f; //0.006f;
	float g = 9.81f;
	float b_p = 0.00001;
	float b_c = 0.1f;
	float e_t = m_p * g * L_p;
	//float e_p = 0.5f*(I_p+m_p*L_p*L_p)*ThetaDot*ThetaDot + m_p*g*L_p*cos(Theta);  // Still something fishy about this one.
	float e_p = m_p*g*L_p*cos(Theta);
	// float e_p = m_p * g * L_p * cos(Theta);
	
	if (Theta < PI + 0.5f && ThetaDot < 0.f)
	{
		SetPoint = 0.3f;
	}
	else if (Theta >= PI - 0.5f && ThetaDot >= 0.f)
	{
		SetPoint = -0.3f;
	}
	else
	{
		SetPoint = Position;
	}

	float Error = SetPoint - Position;
	float u;

	if (Theta >= PI - 1.5f && Theta < PI + 1.5f)
	{
		u = 2.5f*Error*(e_t - e_p) + b_p*ThetaDot;
		// u = 2.f * Error;
	}
	else
	{
		u = 0;
	}
	return u;
}

void FSolver::Stop()
{
	bStopThread = true;
}

void FSolver::GetPose(float& Position, float& Rotation, float& Force)
{
	UE4_Mutex.Lock();
	Position = MoveTemp(CartPosition);
	Rotation = MoveTemp(PoleRotation);
	Force = MoveTemp(CartForce);
	UE4_Mutex.Unlock();
}

void FSolver::GetSetPoint(float& SetPointRef)
{
	UE4_Mutex.Lock();
	SetPointRef = MoveTemp(SetPointIn);
	UE4_Mutex.Unlock();
};


double FSolver::GetElapsedTime()
{
	UE4_Mutex.Lock();
	double Time = ElapsedTime;
	UE4_Mutex.Unlock();
	return Time;
}

float FSolver::SignOfFloat(const float Value)
{
	return Value >= 0.0f ? 1.0f : -1.0f;
}
