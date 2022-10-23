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

	float w = 0.35f;//3.1415f;
	float w_d = 0.0f;
	float w_dd = 0.0f;

	float b_c = 1.15f;
	float b_p = 0.35f;//1.17f;
	// float b_p = 0.0075f;

	float F_m = 0.0f;

	float x_ref = 0.f;
	float x_d_ref = 0.f;
	float w_ref = 0.f;
	float w_d_ref = 0.f;
	float k_gains[4] = {-31.6227766, -51.12423315, 584.51134517, 110.81444504};


	float m_p = 10.f; //0.071f;
	float m_c = 10.f; //0.288f;
	float L_p = 0.6f; //(0.685f - 0.246f);
	float I_p = 0.05f; //0.006f;
	float g = 9.81f;
	float u = 0;
	bool SwingUp = true;
	bool UseLQR = false;
	float e_t = m_p * g * L_p;
	float e_p = 0;
	float e = 0.f;
	float rail_ends = 0.415f;
	
	std::chrono::steady_clock::time_point StepStartTime;
	std::chrono::steady_clock::time_point StepEndTime;
	std::chrono::steady_clock::time_point Timer = std::chrono::steady_clock::now();
	while (!bStopThread)
	{
		StepStartTime = std::chrono::steady_clock::now();

		//F_m = u;

		// if (WorldPtr->GetWorld()->GetRealTimeSeconds() < 2.f)
		// {
		// 	F_m = 0.f;
		// }
		// else if(WorldPtr->GetWorld()->GetRealTimeSeconds() > 2.f &&  WorldPtr->GetWorld()->GetRealTimeSeconds() < 6.f)
		// {
		// 	F_m = 30.f;
		// }
		// else
		// {
		// 	F_m = 0.f;;
		// }


		x_dd = (F_m - b_c * x_d + m_p * L_p * w_dd * cos(w) - m_p * L_p * w_d * w_d * sin(w)) / (m_p + m_c);
		w_dd = (m_p * L_p * g * sin(w) + m_p * L_p * x_dd * cos(w) - b_p * w_d) / (I_p + m_p * L_p * L_p);
		if (x >= rail_ends)
		{
			x_dd = -x_dd;
			w_dd = -w_dd;
			x_d = -(1 + e) * x_d;
			x = rail_ends;
		}
		if (x <= -rail_ends)
		{
			x_dd = -x_dd;
			w_dd = -w_dd;
			x_d = -(1 + e) * x_d;
			x = -rail_ends;
		}
		
		x_d = x_dd * DeltaTime + x_d;
		x = x_d * DeltaTime + x;
		w_d = w_dd * DeltaTime + w_d;
		w = w_d * DeltaTime + w;
		

		if (SwingUp)
		{
			//u = SwingUpControl(w, w_d, x);

			if (w > 1.571 || w < 4.712)
			{
				e_p = 0.5f * I_p * w_d * w_d + m_p * g * L_p * cos(w);
				u = (e_t - e_p) * w_d * cos(w) * 0.18525;
			}
			else if (e_t == e_p)
			{
				u = 0;
			}
			else
			{
				u = 0;
			}


			// if (w < 0.0 + 0.15 || w > 3.1415 * 2 - 0.15)
			// {
			// 	SwingUp = false;
			// 	UseLQR = true;
			// }
		}
		if (UseLQR)
		{
			// X = matrix([
			// 	[x],
			// 	[dx],
			// 	[a],
			// 	[da]
			// ])
			float temp_x = x - x_ref;
			float temp_xd = x_d - x_d_ref;
			float temp_w = w - w_ref;
			float temp_wd = w_d - w_d_ref;

			u = -k_gains[0] * temp_x - k_gains[1] * temp_xd - k_gains[2] * temp_w - k_gains[3] * temp_wd;
			// u = -K*(X - ref);
		}

		if (u > 1000.f)
		{
			u = 1000.f;
		}

		UE4_Mutex.Lock();
		CartPosition = x * 100.0f; // Converting from [m] to [cm]
		PoleRotation = w * 180.0f / 3.14f + 180; // Converting from [rad] to [deg], and rotating to UE axis
		UE4_Mutex.Unlock();

		StepEndTime = std::chrono::steady_clock::now();
		StepTime = std::chrono::duration<float>(StepEndTime - StepStartTime).count();

		if (StepTime < DeltaTime)
		{
			// UE_LOG(LogTemp, Warning, TEXT("StepTime %f[s], sleeping for %f[s]"), StepTime, DeltaTime-StepTime);
			// std::this_thread::sleep_for(std::chrono::duration<double>(DeltaTime - StepTime));
			//FPlatformProcess::Sleep(DeltaTime - StepTime); Sleep all threads
			std::this_thread::sleep_for(std::chrono::duration<double>(DeltaTime - StepTime));
		}
		else
		{
			// UE_LOG(LogTemp, Warning, TEXT("Step period was larger than dt. Time overflow: %f[s]"), StepTime);
		}
		//UE_LOG(LogTemp, Warning, TEXT("Period with wait: %f[s]"), std::chrono::duration<float>(std::chrono::steady_clock::now() - StepStartTime).count());
		ElapsedTime = std::chrono::duration<float>(std::chrono::steady_clock::now() - Timer).count();
	}
	return 0;
}


float FSolver::SwingUpControl(float Theta, float ThetaDot, float Position)
{
	float m_p = 10.f; //0.071f;
	float m_c = 10.f; //0.288f;
	float L_p = 0.6f; //(0.685f - 0.246f);
	float I_p = 0.05f; //0.006f;
	float g = 9.81f;
	float b_p = 1.17;
	float SetPoint;
	float e_t = m_p * g * L_p;
	//float e_p = 0.5f*(I_p+m_p*L_p*L_p)*ThetaDot*ThetaDot + m_p*g*L_p*cos(Theta);
	float e_p = m_p * g * L_p * cos(Theta);
	float Direction = -75.0f * SignOfFloat(ThetaDot) * SignOfFloat(Theta);

	if (Theta > PI)
	{
		SetPoint = -0.3f;
	}
	if (Theta <= PI)
	{
		SetPoint = 0.3f;
	}

	float Error = SetPoint - Position;
	//float u = (e_t - e_p)*ThetaDot*cos(Theta)*0.195;
	float u;
	if (Theta > PI - 0.1 || Theta < PI + 0.1)
	{
		u = Error * Direction;
	}
	else
	{
		u = 0;
	}
	u = -0.1f * ThetaDot * Theta * (e_t - e_p);

	return u;
}


void FSolver::Stop()
{
	bStopThread = true;
}

void FSolver::GetPose(float& Position, float& Rotation)
{
	UE4_Mutex.Lock();
	Position = MoveTemp(CartPosition);
	Rotation = MoveTemp(PoleRotation);
	UE4_Mutex.Unlock();
}

double FSolver::GetElapsedTime()
{
	UE4_Mutex.Lock();
	double Time = ElapsedTime;
	UE4_Mutex.Unlock();
	return Time;
}

float FSolver::SignOfFloat(float Value)
{
	return (Value >= 0.0f) ? 1.0f : -1.0f;
}
