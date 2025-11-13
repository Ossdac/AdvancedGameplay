#include "AG_RigidbodyComponent.h"
#include "GameFramework/Actor.h"

UAG_RigidbodyComponent::UAG_RigidbodyComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UAG_RigidbodyComponent::BeginPlay()
{
	Super::BeginPlay();

	TimeAccumulator = 0.0f;

	// Ensure gravity direction is normalized
	if (!GravityDirection.IsNearlyZero())
	{
		GravityDirection = GravityDirection.GetSafeNormal();
	}
	else
	{
		GravityDirection = FVector(0.0f, 0.0f, -1.0f);
	}
}

void UAG_RigidbodyComponent::TickComponent(
	float DeltaTime,
	enum ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction
)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	TimeAccumulator += DeltaTime;

	const int32 MaxSubSteps = 5;
	int32 NumSteps = 0;

	while (TimeAccumulator >= FixedTimeStep && NumSteps < MaxSubSteps)
	{
		FixedUpdate(FixedTimeStep);
		TimeAccumulator -= FixedTimeStep;
		++NumSteps;
	}
}

void UAG_RigidbodyComponent::FixedUpdate(float FixedDeltaTime)
{
	// if (Mass <= 0.0f)
	// {
	// 	return;
	// }

	// Clear forces at the start of the step
	AccumulatedForces = FVector::ZeroVector;

	// Add gravity
	ApplyGravity(FixedDeltaTime);

	// F = m * a => a = F / m
	const FVector Acceleration = AccumulatedForces / Mass;

	// Semi-implicit Euler integration
	Velocity += Acceleration * FixedDeltaTime;

	// Move the owning Actor
	if (AActor* Owner = GetOwner())
	{
		const FVector NewLocation = Owner->GetActorLocation() + Velocity * FixedDeltaTime;
		Owner->SetActorLocation(NewLocation, false); // no sweep for now, you’ll add collisions later
	}
}

void UAG_RigidbodyComponent::ApplyGravity(float FixedDeltaTime)
{
	// Gravity is a constant acceleration -> treat as force: F = m * g
	const FVector GravityAccel = GravityDirection * GravityStrength;
	const FVector GravityForce = Mass * GravityAccel;

	AccumulatedForces += GravityForce;
}

void UAG_RigidbodyComponent::AddForce(const FVector& Force)
{
	AccumulatedForces += Force;
}
