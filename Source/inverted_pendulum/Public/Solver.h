// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "SystemParameters.h"
#include <chrono>
/**
 * 
 */
class INVERTED_PENDULUM_API FSolver : public FRunnable
{
public:
	explicit FSolver(UWorld* UWorldPtr);
	virtual ~FSolver() override;

	// FRunnable Pure Virtual functions
	virtual bool Init();
	virtual uint32 Run();
	virtual void Stop();

	bool IsRunning() const {return !bStopThread;}
	void GetPose(float& Position, float& Rotation, float& Force);
	void GetSetPoint(float& SetPointRef);
	double GetElapsedTime();

protected:
	FCriticalSection UE4_Mutex;
	FRunnableThread* Thread = nullptr;

private:
	MechanicalParamerters SysParameters; 
	UWorld* WorldPtr;
	bool bStopThread;
	float SetPoint;
	float SetPointIn;
	float CartPosition;
	float PoleRotation;
	float CartForce;
	double ElapsedTime;
	float ErrorIntegral;
	float SwingUpControl(float Theta, float ThetaDot, float Position, float Velocity);
	float SignOfFloat(float Value);
};
