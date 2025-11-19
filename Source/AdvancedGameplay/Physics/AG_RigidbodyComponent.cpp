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
	
	// Pick a default component to move if none assigned
	if (!UpdatedComponent)
	{
		if (AActor* Owner = GetOwner())
		{
			UpdatedComponent = Cast<UPrimitiveComponent>(Owner->GetRootComponent());
			
			if (!UpdatedComponent)
			{
				UpdatedComponent = Owner->FindComponentByClass<UPrimitiveComponent>();
			}
			
			if (!UpdatedComponent)
			{
				GEngine->AddOnScreenDebugMessage(
					-1,
					5.0f,
					FColor::Red,
					TEXT("UAG_RigidbodyComponent: No UpdatedComponent assigned and Owner has no UPrimitiveComponent root!")
				);
			}
		}
	}

	// Make sure physics is not being simulated by Chaos on this component
	if (UpdatedComponent && bEnableCollision)
	{
		UpdatedComponent->SetSimulatePhysics(false);
		UpdatedComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
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

	constexpr int32 MaxSubSteps = 100;
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

	// No component to move? Fallback to actor movement without collision
	if (!UpdatedComponent)
	{
		if (AActor* Owner = GetOwner())
		{
			const FVector NewLocation = Owner->GetActorLocation() + Velocity * FixedDeltaTime;
			Owner->SetActorLocation(NewLocation, false);
		}
		return;
	}

	const FVector Delta = Velocity * FixedDeltaTime;
	if (Delta.IsNearlyZero())
	{
		return;
	}

	FHitResult Hit;
	const FQuat Rotation = UpdatedComponent->GetComponentQuat();

	UpdatedComponent->MoveComponent(Delta, Rotation, bEnableCollision, &Hit);

	if (bEnableCollision && Hit.bBlockingHit)
	{
		HandleBlockingHit(Hit, FixedDeltaTime);
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

void UAG_RigidbodyComponent::HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime)
{
	if (!UpdatedComponent)
	{
		return;
	}

	const FVector Normal = Hit.ImpactNormal.GetSafeNormal();
	const float vDotN = FVector::DotProduct(Velocity, Normal);

	// Only correct if we are moving into the surface
	if (vDotN < 0.0f)
	{
		// Decompose velocity into normal and tangential components
		const FVector vN = vDotN * Normal;        // normal component
		FVector vT = Velocity - vN;               // tangential component

		// Normal response (bounce along normal, rest stays in tangent)
		const FVector vN_after = -Restitution * vN;

		// --- Simple friction model ---
		// Static friction: if tangential speed is very small, stop completely.
		const float TangentSpeed = vT.Size();
		if (TangentSpeed < KINDA_SMALL_NUMBER)
		{
			vT = FVector::ZeroVector;
		}
		else
		{
			// Approximate dynamic friction as scaling down tangential velocity.
			// The stronger the normal impact, the more we slow down along the surface.
			// This is not physically perfect, but good enough for a custom gameplay phys layer.
			const float NormalSpeed = FMath::Abs(vDotN);
			const float FrictionScale = FMath::Clamp(
				1.0f - DynamicFriction * NormalSpeed * FixedDeltaTime,
				0.0f,
				1.0f
			);

			vT *= FrictionScale;

			// Optional: rudimentary static friction threshold
			if (vT.Size() < StaticFriction * NormalSpeed * FixedDeltaTime)
			{
				vT = FVector::ZeroVector;
			}
		}

		Velocity = vT + vN_after;
	}

	// Small depenetration to help avoid staying embedded due to numerical issues
	const float PenetrationSlack = 0.1f;
	UpdatedComponent->AddWorldOffset(Normal * PenetrationSlack, false);
}

void UAG_RigidbodyComponent::SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent)
{
	if (bEnableCollision && NewUpdatedComponent)
	{
		NewUpdatedComponent->SetSimulatePhysics(false);
		NewUpdatedComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		UpdatedComponent = NewUpdatedComponent;
	}
}

