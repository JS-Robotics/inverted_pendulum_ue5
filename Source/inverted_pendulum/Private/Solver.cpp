// Fill out your copyright notice in the Description page of Project Settings.


#include "Solver.h"

FSolver::FSolver(UWorld* UWorldPtr): WorldPtr(UWorldPtr)
{
	Thread = FRunnableThread::Create(this, TEXT("Solver Thread"));
	const float WorldTime = WorldPtr->GetWorld()->GetRealTimeSeconds();
	UE_LOG(LogTemp, Warning, TEXT("Solver Thread GetWorld.GetTime() =  %f"), WorldTime);
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
	bStopThread = false;
	float DeltaTime = 0.00015;
	float StepTime = 0.0f;

	float x = 0.0f;
	float x_d = 0.0f;
	float x_dd = 0.0f;

	float w = 3.12f/2.0f;
	float w_d = 0.0f;
	float w_dd = 0.0f;

	float b = 0.0f;

	float F_m = 0.0f;

	float m_p = 0.071f;
	float m_c = 0.288f;
	float L_p = (0.685f - 0.246f);
	float I_p = 0.006f;
	float g = 9.81f;
	
	float StepStartTime = 0.0f;

	float u = 0.0f;
	float d = 0.0f;
	
	while (!bStopThread)
	{
		StepStartTime = WorldPtr->GetWorld()->GetRealTimeSeconds();

		// F_m = 0.01f*sin(StepStartTime);
	
		// x_dd = (m_p*m_p*L_p*L_p*g*cos(w)*sin(w) + m_p*L_p*L_p*(m_p*L_p*w_d*w_d*sin(w) - d*x_d) + m_p*L_p*L_p*u) / (m_p*L_p*L_p* (m_c + m_p*(1-cos(w)*cos(w))));
		// w_dd = (-(m_p+m_c)*m_p*g*L_p*sin(w) - m_p*L_p*cos(w)*(m_p*L_p*w_d*w_d*sin(w)- d*x_d) + m_p*L_p*cos(w)*u) / (m_p*L_p*L_p*(m_c+m_p*(1-cos(w)*cos(w))));
		
		// x_dd = (F_m - b * x_d + m_p * L_p * w_dd * cos(w) - m_p * L_p * w_d * w_d * sin(w)) / (m_p + m_c);
		// w_dd = (m_p * L_p * g * sin(w) + m_p * L_p * x_dd * cos(w)) / (I_p + m_p * L_p * L_p);

		// From site
		x_dd = (F_m - b * x_d + m_p * L_p * w_dd * cos(w) - m_p * L_p * w_d * w_d * sin(w)) / (m_p + m_c);
		w_dd = (-m_p * L_p * g * sin(w) - m_p * L_p * x_dd * cos(w)) / (I_p + m_p * L_p * L_p);
		
		x_d = x_dd * DeltaTime + x_d;
		w_d = w_dd * DeltaTime + w_d;
		
		x = 0.5 * DeltaTime * DeltaTime * x_dd + x_d * DeltaTime + x;
		w = 0.5 * DeltaTime * DeltaTime * w_dd + w_d * DeltaTime + w;

		UE4_Mutex.Lock();
		CartPosition = x * 10.0f; // Converting from [m] to [cm]
		PoleRotation = w * 180.0f / 3.14f; // Converting from [rad] to [deg]
		UE4_Mutex.Unlock();
		StepTime = WorldPtr->GetWorld()->GetRealTimeSeconds() - StepStartTime;

		if (StepTime < DeltaTime)
		{
			FPlatformProcess::Sleep(DeltaTime - StepTime);
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("Step period was larger than dt. Time overflow: %f[s]"), (DeltaTime));
		}
		UE_LOG(LogTemp, Warning, TEXT("Angle: %f[rad]"), (w));
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
