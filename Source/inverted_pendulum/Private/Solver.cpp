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
	float DeltaTime;
	const float TimeStep = 0.008333f;
	while (!bStopThread)
	{
		float WorldTime = WorldPtr->GetWorld()->GetRealTimeSeconds();
		UE4_Mutex.Lock();
		CartPosition = 10.0f*sin(WorldTime);
		PoleRotation = 180.0f*sin(WorldTime);
		UE4_Mutex.Unlock();
		DeltaTime = WorldPtr->GetWorld()->GetRealTimeSeconds() - WorldTime;
		if (DeltaTime < TimeStep)
		{
			FPlatformProcess::Sleep(TimeStep - DeltaTime);
		}
	}
	UE_LOG(LogTemp, Warning, TEXT("Thread Finished: !bStopThread: %s"), (!bStopThread ? TEXT("true") : TEXT("false")));
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
