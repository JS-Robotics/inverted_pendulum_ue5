// Fill out your copyright notice in the Description page of Project Settings.


#include "InvertedPendulum.h"

// Sets default values
AInvertedPendulum::AInvertedPendulum()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	static ConstructorHelpers::FObjectFinder<UStaticMesh> BaseMesh(TEXT("/Game/Meshes/SM_Base.SM_Base"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CartMesh(TEXT("/Game/Meshes/SM_Cart.SM_Cart"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> PendulumMesh(TEXT("/Game/Meshes/SM_Pendulum.SM_Pendulum"));

	BaseStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base Mesh")); // Create Object
	BaseStaticMesh->SetStaticMesh(BaseMesh.Object);
	BaseStaticMesh->SetupAttachment(RootComponent);

	CartStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Cart Mesh"));
	CartStaticMesh->SetStaticMesh(CartMesh.Object);
	CartStaticMesh->SetupAttachment(BaseStaticMesh);
	CartStaticMesh->SetRelativeLocation({0.0f, 7.2269f, 91.609}); // Set position in relation to base_mesh origin [cm].

	// Create a dummy link to act as a revolute joint between the cart_mesh and the pole_mesh
	RevoluteJoint = CreateDefaultSubobject<USceneComponent>(TEXT("This is a joint"));
	RevoluteJoint->SetupAttachment(CartStaticMesh); // Make the cart the "parent link"

	// Pole
	PoleStaticMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Pole Mesh"));
	PoleStaticMesh->SetStaticMesh(PendulumMesh.Object); // Add the statich mesh PendulumMesh
	PoleStaticMesh->SetupAttachment(RevoluteJoint); // Attach it to the revolute joint as "child link"
	PoleStaticMesh->SetRelativeLocation({0.0f, 0.0f, 0.0f}); // Set position in relation to revolute_joint origin [cm].
}

// Called when the game starts or when spawned
void AInvertedPendulum::BeginPlay()
{
	Super::BeginPlay();

	CartStaticMesh->SetRelativeLocation({0.0f, 7.2269f, 91.609f}); // Reset back position on begin play
	PoleStaticMesh->SetRelativeLocation({0.0f, 0.0f, 0.0f}); // Reset back position on begin play
}

// Called every frame
void AInvertedPendulum::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}
