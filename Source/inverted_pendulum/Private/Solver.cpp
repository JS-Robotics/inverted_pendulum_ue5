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
	DeltaTime = 1.0f/200.0f;
	float StepTime = 0.0f;

	float x = 0.0f;
	float x_d = 0.0f;
	float x_dd = 0.0f;

	float w = 0.01f;
	float w_d = 0.0f;
	float w_dd = 0.0f;

	float b_c = 0.00000005f;
	float b_p = 0.06f;
	// float b_p = 0.0075f;

	float F_m = 0.0f;

	float m_p = 10.f; //0.071f;
	float m_c = 10.f; //0.288f;
	float L_p = 0.6f; //(0.685f - 0.246f);
	float I_p = 0.05f; //0.006f;
	float g = 9.81f;
	float u = 0;
	
	std::chrono::steady_clock::time_point StepStartTime;
	std::chrono::steady_clock::time_point StepEndTime;
	std::chrono::steady_clock::time_point Timer = std::chrono::steady_clock::now();
	float set_point = 0.f;
	float error = 0;
	while (!bStopThread)
	{
		StepStartTime = std::chrono::steady_clock::now();
		
		x_dd = (F_m - b_c * x_d + m_p * L_p * w_dd * cos(w) - m_p * L_p * w_d * w_d * sin(w)) / (m_p + m_c);
		w_dd = (m_p * L_p * g * sin(w) + m_p * L_p * x_dd * cos(w) - b_p * w_d) / (I_p + m_p * L_p * L_p);
		
		w_d = w_dd * DeltaTime + w_d;
		x_d = x_dd * DeltaTime + x_d;
		
		w = 0.5 * DeltaTime * DeltaTime * w_dd + w_d * DeltaTime + w;
		x = 0.5 * DeltaTime * DeltaTime * x_dd + x_d * DeltaTime + x;
		
		UE4_Mutex.Lock();
		CartPosition = x * 10.0f; // Converting from [m] to [cm]
		PoleRotation = w * 180.0f / 3.14f + 180; // Converting from [rad] to [deg], and rotating to UE axis
		UE4_Mutex.Unlock();

		StepEndTime = std::chrono::steady_clock::now();
		StepTime = std::chrono::duration<float>( StepEndTime - StepStartTime).count();
		
		if (StepTime < DeltaTime)
		{
			// UE_LOG(LogTemp, Warning, TEXT("StepTime %f[s], sleeping for %f[s]"), StepTime, DeltaTime-StepTime);
			// std::this_thread::sleep_for(std::chrono::duration<double>(DeltaTime - StepTime));
			FPlatformProcess::Sleep(DeltaTime - StepTime);
		}
		else
		{
			// UE_LOG(LogTemp, Warning, TEXT("Step period was larger than dt. Time overflow: %f[s]"), StepTime);
		}
		UE_LOG(LogTemp, Warning, TEXT("Period with wait: %f[s]"), std::chrono::duration<float>(std::chrono::steady_clock::now() - StepStartTime).count());
		ElapsedTime = std::chrono::duration<float>(std::chrono::steady_clock::now() - Timer).count();
	}
	return 0;
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