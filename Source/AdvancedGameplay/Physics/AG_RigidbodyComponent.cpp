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
	
	if (InertiaScalar <= 0.0f && UpdatedComponent)
	{
		const float Radius = UpdatedComponent->Bounds.SphereRadius; // good enough
		InertiaScalar = 0.4f * Mass * Radius * Radius; // solid sphere approx
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
	// Skip simulation if sleeping
	// TODO: No need to check bCanSleep here after debugging done
	if (bSleeping && bCanSleep)
	{
		return;
	}
	// Only for debugging purposes: if sleeping is disabled, never sleep
	if (!bCanSleep)
	{
		bSleeping = false;
	}

	bIsGrounded = false;

	IntegrateForces();
	IntegrateVelocity(FixedDeltaTime);
	IntegrateAngularVelocity(FixedDeltaTime);

	DoMovementAndCollisions(FixedDeltaTime);
	ApplyRotation(FixedDeltaTime);

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
		
		HandleBlockingHit(SlideHit, SlideDeltaTime);

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
	if (!UpdatedComponent || Mass <= 0.0f)
	{
		return;
	}

	FAGContactData Contact;
	if (!BuildContactData(Hit, Contact))
	{
		return;
	}

	// Grounded classification
	const float UpDot = FVector::DotProduct(-GravityDirection, Contact.Normal);
	bIsGrounded = (UpDot >= GroundNormalCosThreshold);

	// Threshold for "real" impact into the surface
	const float NormalImpactThreshold = 1.0f; // cm/s, tune if needed

	// If we're significantly moving into the surface -> do a full normal impulse
	if (Contact.VRelN < -NormalImpactThreshold)
	{
		ApplyNormalImpulse(Contact);
	}

	// Always try friction when we have tangential slip (impact or resting contact)
	ApplyFrictionImpulse(Contact, FixedDeltaTime, UpDot);

	// Small normal clamp to kill jitter
	ClampContactNormalRestVelocity(Contact);
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

void UAG_RigidbodyComponent::ApplyRotation(float FixedDeltaTime)
{
	if (!bEnableRotation || !UpdatedComponent)
	{
		return;
	}

	const float Angle = AngularVelocity.Size() * FixedDeltaTime;
	if (Angle <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector Axis = AngularVelocity.GetSafeNormal();
	const FQuat   DeltaRot(Axis, Angle);
	UpdatedComponent->AddWorldRotation(DeltaRot, false);
}

bool UAG_RigidbodyComponent::BuildContactData(
	const FHitResult& Hit,
	FAGContactData& OutData
) const
{
	if (!UpdatedComponent || Mass <= 0.0f)
	{
		return false;
	}

	// Normal
	const FVector Normal = Hit.ImpactNormal.GetSafeNormal();
	if (Normal.IsNearlyZero())
	{
		return false;
	}

	const FVector ComPos = UpdatedComponent->GetComponentLocation();

	// Contact point: prefer ImpactPoint, fall back to COM if degenerate
	FVector ContactPoint = Hit.ImpactPoint;
	if (ContactPoint.IsNearlyZero())
	{
		ContactPoint = ComPos;
	}

	const FVector R = ContactPoint - ComPos;

	// Contact velocity: v_c = v + ω × r
	const FVector VContact = Velocity + FVector::CrossProduct(AngularVelocity, R);

	const float VRelN = FVector::DotProduct(VContact, Normal);

	OutData.Normal       = Normal;
	OutData.ContactPoint = ContactPoint;
	OutData.R            = R;
	OutData.VContact     = VContact;
	OutData.VRelN        = VRelN;
	OutData.NormalImpulse = 0.0f;

	return true;
}


void UAG_RigidbodyComponent::ApplyNormalImpulse(FAGContactData& Contact)
{
	// v_rel_n_before
	const float vRelN = Contact.VRelN;

	// Desired post-impact normal relative velocity: v'_rel_n = -e v_rel_n
	const float e = FMath::Clamp(Restitution, 0.0f, 1.0f);
	const float desiredRelN = -e * vRelN;
	const float deltaRelN   = desiredRelN - vRelN; // = -(1+e) * vRelN

	// Effective inverse mass along the normal:
	// 1/m + ((r × n)^2 / I) for scalar inertia
	const FVector rCrossN      = FVector::CrossProduct(Contact.R, Contact.Normal);
	const float   rCrossNLenSq = rCrossN.SizeSquared();

	float invEffMassN = 1.0f / Mass;
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		invEffMassN += rCrossNLenSq / InertiaScalar;
	}

	if (invEffMassN <= SMALL_NUMBER)
	{
		Contact.NormalImpulse = 0.0f;
		return;
	}

	// Scalar normal impulse
	float jN = deltaRelN / invEffMassN;

	// Only allow impulse pushing us out of the surface
	if (jN < 0.0f)
	{
		jN = 0.0f;
	}

	Contact.NormalImpulse = jN;

	if (jN == 0.0f)
	{
		return;
	}

	const FVector ImpulseN = jN * Contact.Normal;

	// Linear
	Velocity += ImpulseN / Mass;

	// Angular
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		const FVector dOmegaN = FVector::CrossProduct(Contact.R, ImpulseN) / InertiaScalar;
		AngularVelocity += dOmegaN;
	}
}


void UAG_RigidbodyComponent::ApplyFrictionImpulse(
	FAGContactData& Contact,
	float FixedDeltaTime,
	float UpDot
)
{
	if (DynamicFrictionCoeff <= 0.0f || Mass <= 0.0f)
	{
		return;
	}

	// --------------------------------------------------------------------
	// 1) Decide what "normal impulse" we use as the Coulomb cap
	// --------------------------------------------------------------------

	// Case A: impact frame – we already computed a real normal impulse
	float EffectiveNormalImpulse = Contact.NormalImpulse;

	// Case B: no impact impulse (resting / sliding contact), but we're grounded.
	// Approximate normal impulse for this step as m * g * dt * UpDot.
	if (EffectiveNormalImpulse <= SMALL_NUMBER && bIsGrounded && UpDot > 0.0f)
	{
		const float NormalForce = Mass * GravityStrength * UpDot;           // ~ N
		EffectiveNormalImpulse = NormalForce * FixedDeltaTime;              // N·s (impulse)
	}

	if (EffectiveNormalImpulse <= SMALL_NUMBER)
	{
		// No meaningful normal support -> no friction (Coulomb model)
		return;
	}

	// --------------------------------------------------------------------
	// 2) Compute tangential relative velocity at the contact
	// --------------------------------------------------------------------
	const FVector VContact =
		Velocity + FVector::CrossProduct(AngularVelocity, Contact.R);

	const float   vRelN = FVector::DotProduct(VContact, Contact.Normal);
	const FVector vT    = VContact - vRelN * Contact.Normal;
	const float   vTLen = vT.Size();

	if (vTLen <= KINDA_SMALL_NUMBER)
	{
		// No slip -> no dynamic friction
		return;
	}

	const FVector TangentDir = vT / vTLen;

	// Effective inverse mass along tangent
	const FVector rCrossT      = FVector::CrossProduct(Contact.R, TangentDir);
	const float   rCrossTLenSq = rCrossT.SizeSquared();

	float invEffMassT = 1.0f / Mass;
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		invEffMassT += rCrossTLenSq / InertiaScalar;
	}

	if (invEffMassT <= SMALL_NUMBER)
	{
		return;
	}

	// --------------------------------------------------------------------
	// 3) Impulse that tries to kill tangential velocity, but Coulomb-clamped
	// --------------------------------------------------------------------

	// j_t that would zero v_t
	float jT = -vTLen / invEffMassT;

	// Coulomb limit: |j_t| ≤ μ * EffectiveNormalImpulse
	const float maxFrictionImpulse = DynamicFrictionCoeff * EffectiveNormalImpulse;
	jT = FMath::Clamp(jT, -maxFrictionImpulse, maxFrictionImpulse);

	if (FMath::IsNearlyZero(jT))
	{
		return;
	}

	const FVector ImpulseT = jT * TangentDir;

	// Apply linear
	Velocity += ImpulseT / Mass;

	// Apply angular
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		const FVector dOmegaT = FVector::CrossProduct(Contact.R, ImpulseT) / InertiaScalar;
		AngularVelocity += dOmegaT;
	}
}



void UAG_RigidbodyComponent::ClampContactNormalRestVelocity(FAGContactData& Contact)
{
	// Recompute contact velocity with both impulses applied
	const FVector VContactFinal =
		Velocity + FVector::CrossProduct(AngularVelocity, Contact.R);

	const float vRelN_final = FVector::DotProduct(VContactFinal, Contact.Normal);

	const float NormalRestThreshold = 1.0f; // cm/s, tune to taste
	if (FMath::Abs(vRelN_final) < NormalRestThreshold)
	{
		// Approx: remove normal component from linear velocity only
		Velocity -= vRelN_final * Contact.Normal;
	}
}

