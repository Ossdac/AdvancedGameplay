#include "AG_RigidbodyComponent.h"
#include "GameFramework/Actor.h"

UAG_RigidbodyComponent::UAG_RigidbodyComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
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


void UAG_RigidbodyComponent::SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent)
{
	if (bEnableCollision && NewUpdatedComponent)
	{
		NewUpdatedComponent->SetSimulatePhysics(false);
		NewUpdatedComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
		UpdatedComponent = NewUpdatedComponent;
	}
}

void UAG_RigidbodyComponent::AddForce(const FVector& Force)
{
	if (!Force.IsNearlyZero())
	{
		// Any explicit external force wakes the body
		bSleeping = false;
		FramesAtRest = 0;
	}

	AccumulatedForces += Force;
}

void UAG_RigidbodyComponent::AddTorque(const FVector& Torque)
{
	if (!Torque.IsNearlyZero())
	{
		// Any explicit torque wakes the body
		bSleeping    = false;
		FramesAtRest = 0;
	}

	AccumulatedTorque += Torque;
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
					TEXT(
						"UAG_RigidbodyComponent: No UpdatedComponent assigned and Owner has no UPrimitiveComponent root!")
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


void UAG_RigidbodyComponent::FixedUpdate(float FixedDeltaTime)
{
	if (bSleeping)
	{
		return;
	}

	bIsGrounded = false;

	IntegrateForces();
	IntegrateVelocity(FixedDeltaTime);
	IntegrateAngularVelocity(FixedDeltaTime);

	DoMovementAndCollisions(FixedDeltaTime);

	if (bEnableRotation && UpdatedComponent)
	{
		const float Angle = AngularVelocity.Size() * FixedDeltaTime;
		if (Angle > KINDA_SMALL_NUMBER)
		{
			const FVector Axis = AngularVelocity.GetSafeNormal();
			const FQuat   DeltaRot(Axis, Angle);
			UpdatedComponent->AddWorldRotation(DeltaRot, false);
		}
	}

	UpdateSleepState();

	AccumulatedForces = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
}



void UAG_RigidbodyComponent::IntegrateForces()
{
	ApplyGravity();
	ApplyDragForce();
	ApplyAngularDrag();
}

void UAG_RigidbodyComponent::IntegrateVelocity(float FixedDeltaTime)
{
	// F = m * a => a = F / m
	const FVector Acceleration = AccumulatedForces / Mass;

	// Semi-implicit Euler integration
	Velocity += Acceleration * FixedDeltaTime;
}

void UAG_RigidbodyComponent::IntegrateAngularVelocity(float FixedDeltaTime)
{
	if (!bEnableRotation || InertiaScalar <= 0.0f)
	{
		return;
	}

	// α = τ / I
	const FVector AngularAcceleration = AccumulatedTorque / InertiaScalar;

	// Semi-implicit Euler for rotation: ω_{n+1} = ω_n + α Δt
	AngularVelocity += AngularAcceleration * FixedDeltaTime;
}

void UAG_RigidbodyComponent::DoMovementAndCollisions(float FixedDeltaTime)
{
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

	float RemainingTime = 1.0f;

	for (int32 Iteration = 0; Iteration < MaxContactIterations && RemainingTime > KINDA_SMALL_NUMBER; ++Iteration)
	{
		const float StepDeltaTime = FixedDeltaTime * RemainingTime;
		const FVector Delta = Velocity * StepDeltaTime;

		if (Delta.IsNearlyZero())
		{
			if (Iteration > 0)
				GEngine->AddOnScreenDebugMessage(
					-1,
					1.0f,
					FColor::Green,
					FString::Printf(TEXT("Delta near zero, exit at Rigidbody Iterations: %d"), Iteration)
				);
			break;
		}

		FHitResult Hit;
		const FQuat Rotation = UpdatedComponent->GetComponentQuat();

		// Sweep along full remaining Delta this iteration
		UpdatedComponent->MoveComponent(Delta, Rotation, bEnableCollision, &Hit);

		if (!bEnableCollision || !Hit.bBlockingHit)
		{
			if (Iteration > 0)
				GEngine->AddOnScreenDebugMessage(
					-1,
					1.0f,
					FColor::Green,
					FString::Printf(TEXT("No blocking hit, exit at Rigidbody Iterations: %d"), Iteration)
				);
			// Moved freely, no more contacts this step
			break;
		}

		// Time of impact for this sweep, fraction of RemainingTime
		const float HitTime = Hit.Time; // in [0,1]

		// Consume the fraction of time up to contact
		RemainingTime *= (1.0f - HitTime);

		// Resolve velocity at contact (bounce, friction, etc.)
		HandleBlockingHit(Hit, StepDeltaTime);

		// If no time left in this step, we are done
		if (RemainingTime <= KINDA_SMALL_NUMBER)
		{
			if (Iteration > 0)
				GEngine->AddOnScreenDebugMessage(
					-1,
					1.0f,
					FColor::Green,
					FString::Printf(TEXT("No remaining time, exit at Rigidbody Iterations: %d"), Iteration)
				);
			break;
		}

		// Slide for the remaining time along the contact plane
		const FVector Normal = Hit.ImpactNormal.GetSafeNormal();
		const float SlideDeltaTime = FixedDeltaTime * RemainingTime;
		const FVector NewDelta = Velocity * SlideDeltaTime;
		const FVector SlideDelta = FVector::VectorPlaneProject(NewDelta, Normal);

		if (SlideDelta.IsNearlyZero())
		{
			if (Iteration > 0)
				GEngine->AddOnScreenDebugMessage(
					-1,
					1.0f,
					FColor::Green,
					FString::Printf(TEXT("Slide delta near zero, exit at Rigidbody Iterations: %d"), Iteration)
				);
			// No slide movement, done for this step
			break;
		}

		FHitResult SlideHit;
		UpdatedComponent->MoveComponent(SlideDelta, Rotation, bEnableCollision, &SlideHit);

		if (!bEnableCollision || !SlideHit.bBlockingHit)
		{
			if (Iteration > 0)
				GEngine->AddOnScreenDebugMessage(
					-1,
					1.0f,
					FColor::Green,
					FString::Printf(TEXT("Slide no blocking hit, exit at Rigidbody Iterations: %d"), Iteration)
				);
			// Slid freely, done for this step
			break;
		}

		// Hit something while sliding: small depenetration and let next iteration
		// treat this (new) contact as the primary hit.
		const FVector SlideNormal = SlideHit.ImpactNormal.GetSafeNormal();
		UpdatedComponent->AddWorldOffset(SlideNormal * UE_SMALL_NUMBER, false);
	}
}

void UAG_RigidbodyComponent::ApplyGravity()
{
	// Gravity is a constant acceleration -> treat as force: F = m * g
	const FVector GravityAccel = GravityDirection * GravityStrength;
	const FVector GravityForce = Mass * GravityAccel;

	AccumulatedForces += GravityForce;
}

void UAG_RigidbodyComponent::ApplyDragForce()
{
	const float Speed = Velocity.Size();
	if (Speed < KINDA_SMALL_NUMBER)
	{
		return;
	}
	// Linear drag: F = -k1 * v
	FVector DragForce = -LinearDragCoeff * Velocity;

	// Quadratic drag: F = -k2 * |v| * v
	if (QuadraticDragCoeff > 0.0f)
	{
		DragForce += -QuadraticDragCoeff * Speed * Velocity;
	}

	AccumulatedForces += DragForce;
}

void UAG_RigidbodyComponent::ApplyAngularDrag()
{
	if (!bEnableRotation || AngularDragCoeff <= 0.0f)
	{
		return;
	}

	const float Omega = AngularVelocity.Size();
	if (Omega < KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Very simple linear angular drag: τ = -c * ω
	const FVector DragTorque = -AngularDragCoeff * AngularVelocity;
	AccumulatedTorque += DragTorque;
}

void UAG_RigidbodyComponent::HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime)
{
	if (!UpdatedComponent)
	{
		return;
	}

	// Contact normal (unit, pointing out of the surface)
	const FVector Normal = Hit.ImpactNormal.GetSafeNormal();

	const float UpDot = FVector::DotProduct(-GravityDirection, Normal);

	bIsGrounded = (UpDot >= GroundNormalCosThreshold);

	// Velocity along the normal
	const float vDotN = FVector::DotProduct(Velocity, Normal);

	// Only resolve if we are moving *into* the surface
	if (vDotN >= 0.0f)
	{
		return;
	}

	// Decompose velocity into normal and tangential components
	const FVector vN = vDotN * Normal; // normal component (into the surface, vDotN < 0)
	FVector vT = Velocity - vN; // tangential (in the contact plane)

	// --- Normal response (restitution) ---
	// vN_after = -e * vN, where e = Restitution (0 = inelastic, 1 = elastic)
	const FVector vN_after = -Restitution * vN;

	// --- Tangential response: simple Coulomb-like dynamic + static friction ---

	// Dynamic friction:
	// Approximate a friction deceleration of a_f ≈ μ_k * g in cm/s^2.
	// Here we use GravityStrength as "g" scale (980 ≈ 1g in Unreal units).
	const float TangentSpeed = vT.Size();
	if (TangentSpeed > UE_SMALL_NUMBER)
	{
		const float FrictionAccel = DynamicFrictionCoeff * GravityStrength; // cm/s^2
		const float MaxSpeedDrop = FrictionAccel * FixedDeltaTime; // cm/s this step

		if (TangentSpeed <= MaxSpeedDrop)
		{
			// Friction can stop tangential motion this step
			vT = FVector::ZeroVector;
		}
		else
		{
			// Reduce tangential speed by MaxSpeedDrop along tangent direction
			const FVector TangentDir = vT / TangentSpeed;
			vT -= TangentDir * MaxSpeedDrop;
		}
	}

	// Static friction snap near zero:
	// Treat StaticFriction as a small speed threshold in cm/s for now.
	if (vT.Size() < StaticFrictionSpeedThreshold)
	{
		vT = FVector::ZeroVector;
	}

	// Recombine to new velocity
	Velocity = vT + vN_after;

	// --- Resting contact normal clamp to kill tiny jitter ---
	const float NewVDotN = FVector::DotProduct(Velocity, Normal);
	const float NormalRestThreshold = 1.0f; // cm/s; tune to taste

	if (FMath::Abs(NewVDotN) < NormalRestThreshold)
	{
		// Remove tiny normal motion completely so we don't "buzz" on the surface
		Velocity -= NewVDotN * Normal;
	}
}


void UAG_RigidbodyComponent::UpdateSleepState()
{
	if (!bCanSleep)
	{
		return;
	}
	const float SpeedSq = Velocity.SizeSquared();
	const float SleepSpeedSq = SleepLinearSpeedThreshold * SleepLinearSpeedThreshold;

	if (bIsGrounded && SpeedSq < SleepSpeedSq)
	{
		// Candidate for sleep
		FramesAtRest++;

		if (FramesAtRest >= MinFramesAtRest)
		{
			// Snap to full rest
			Velocity = FVector::ZeroVector;
			bSleeping = true;
			// Keep FramesAtRest as-is or clamp; doesn't really matter at this point
		}
	}
	else
	{
		// Moving or not grounded -> stay awake
		FramesAtRest = 0;
		bSleeping = false;
	}
}

void UAG_RigidbodyComponent::ApplyAngularSleepClamp()
{
}
