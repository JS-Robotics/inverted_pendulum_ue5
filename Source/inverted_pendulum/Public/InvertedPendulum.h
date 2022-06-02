// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "InvertedPendulum.generated.h"

UCLASS()
class INVERTED_PENDULUM_API AInvertedPendulum : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AInvertedPendulum();

	UPROPERTY(VisibleAnyWhere, BlueprintReadOnly, Category = "CartPole")
	UStaticMeshComponent* BaseStaticMesh;

	UPROPERTY(VisibleAnyWhere, BlueprintReadOnly, Category = "CartPole")
	UStaticMeshComponent* PoleStaticMesh;

	UPROPERTY(VisibleAnyWhere, BlueprintReadOnly, Category = "CartPole")
	UStaticMeshComponent* CartStaticMesh;

	UPROPERTY(VisibleAnyWhere, BlueprintReadOnly, Category = "CartPole")
	USceneComponent* RevoluteJoint;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
