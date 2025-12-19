#include "AG_RigidbodyComponent.h"

#include "GameFramework/Actor.h"

UAG_RigidbodyComponent::UAG_RigidbodyComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UAG_RigidbodyComponent::SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent)
{
	if (!NewUpdatedComponent)
	{
		return;
	}

	NewUpdatedComponent->SetSimulatePhysics(false);
	NewUpdatedComponent->SetCollisionEnabled(bEnableCollision ? ECollisionEnabled::QueryAndPhysics : ECollisionEnabled::NoCollision);
	UpdatedComponent = NewUpdatedComponent;
	CachedRadius = UpdatedComponent->Bounds.SphereRadius;
}


void UAG_RigidbodyComponent::SetGravityDirection(
	const FVector& NewGravityDirection,
	float NewGravityStrength,
	bool bWakeUp
)
{
	// Reject zero input direction
	if (NewGravityDirection.IsNearlyZero())
	{
		return;
	}

	GravityDirection = NewGravityDirection.GetSafeNormal();
	
	// Optionally update gravity strength
	if (NewGravityStrength > 0.0f)
	{
		GravityStrength = NewGravityStrength;
	}

	// Optionally wake the body
	if (bWakeUp)
	{
		WakeUp();
	}
}

void UAG_RigidbodyComponent::AdjustGravityFromContactNormal(bool bWakeUp)
{
	if (!bHasLastGroundNormal)
	{
		return;
	}

	// Gravity should point "down", opposite the ground normal
	const FVector NewGravityDir = -LastGroundNormal;

	// Reuse the existing helper (keeps strength, optional wakeup and normalization)
	SetGravityDirection(NewGravityDir, GravityStrength, bWakeUp);
}

void UAG_RigidbodyComponent::SetAdjustGravityToGround(bool bEnable)
{
	bAdjustGravityToGround = bEnable;
}

bool UAG_RigidbodyComponent::GetAdjustGravityToGround() const
{
	return bAdjustGravityToGround;
}

void UAG_RigidbodyComponent::AddForce(const FVector& Force)
{
	if (!Force.IsNearlyZero())
	{
		WakeUp();
	}

	AccumulatedForces += Force;
}

void UAG_RigidbodyComponent::AddTorque(const FVector& Torque)
{
	if (!Torque.IsNearlyZero())
	{
		WakeUp();
	}

	AccumulatedTorque += Torque;
}

void UAG_RigidbodyComponent::SetRotationEnabled(bool bEnable, bool bClearSpin)
{
	bEnableRotation = bEnable;

	if (bClearSpin && (!AngularVelocity.IsNearlyZero() || !AccumulatedTorque.IsNearlyZero()))
	{
		ClearSpin();
	}
}

void UAG_RigidbodyComponent::ClearSpin()
{
	AngularVelocity = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
	WakeUp();
}

void UAG_RigidbodyComponent::WakeUp()
{
	bSleeping    = false;
	FramesAtRest = 0;
}

void UAG_RigidbodyComponent::ForceSleep()
{
	// Zero all motion
	Velocity = FVector::ZeroVector;
	AngularVelocity = FVector::ZeroVector;

	// Clear accumulators
	AccumulatedForces = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;

	// Reset contact / grounded state
	bIsGrounded = false;
	bHasLastGroundNormal = false;

	// Enter sleep immediately
	bSleeping = true;
	FramesAtRest = MinFramesAtRest;
}

void UAG_RigidbodyComponent::StopMotion(bool bClearForces, bool bClearTorques, bool bWakeUp)
{
	Velocity = FVector::ZeroVector;
	AngularVelocity = FVector::ZeroVector;

	if (bClearForces)
	{
		AccumulatedForces = FVector::ZeroVector;
	}
	if (bClearTorques)
	{
		AccumulatedTorque = FVector::ZeroVector;
	}

	if (bWakeUp)
	{
		WakeUp();
	}
}

void UAG_RigidbodyComponent::AddDriveInput(float ForwardInput, float RightInput, float MaxAccelerationCm, float MaxSpeedCm)
{
	if (Mass <= SMALL_NUMBER)
	{
		return;
	}

	const float ClampedForward = FMath::Clamp(ForwardInput, -1.0f, 1.0f);
	const float ClampedRight   = FMath::Clamp(RightInput,   -1.0f, 1.0f);

	FVector Input = FVector(ClampedForward, ClampedRight, 0.0f);
	if (Input.IsNearlyZero())
	{
		return;
	}
	Input = Input.GetSafeNormal();

	if (!UpdatedComponent)
	{
		return;
	}

	const FVector Up = -GravityDirection.GetSafeNormal();

	// Build a planar basis from the component orientation
	FVector Fwd = UpdatedComponent->GetForwardVector();
	FVector Rgt = UpdatedComponent->GetRightVector();

	Fwd = FVector::VectorPlaneProject(Fwd, Up).GetSafeNormal();
	Rgt = FVector::VectorPlaneProject(Rgt, Up).GetSafeNormal();

	if (Fwd.IsNearlyZero() || Rgt.IsNearlyZero())
	{
		return;
	}

	const FVector WishDir = (Fwd * Input.X + Rgt * Input.Y).GetSafeNormal();
	if (WishDir.IsNearlyZero())
	{
		return;
	}

	// Speed cap (only blocks further acceleration in the same direction)
	if (MaxSpeedCm > 0.0f)
	{
		const FVector PlanarV = GetPlanarVelocity();
		const float PlanarSpeed = PlanarV.Size();

		if (PlanarSpeed >= MaxSpeedCm)
		{
			const float AlongWish = FVector::DotProduct(PlanarV, WishDir);
			if (AlongWish > 0.0f)
			{
				return; // already at/over cap, and trying to accelerate further
			}
		}
	}

	const float Accel = FMath::Max(0.0f, MaxAccelerationCm);
	if (Accel <= SMALL_NUMBER)
	{
		return;
	}

	const FVector Force = WishDir * (Accel * Mass);
	AddForce(Force);
}

void UAG_RigidbodyComponent::AddRollTorqueInput(float ForwardInput, float RightInput, float MaxTorque)
{
	const float ClampedForward = FMath::Clamp(ForwardInput, -1.0f, 1.0f);
	const float ClampedRight   = FMath::Clamp(RightInput,   -1.0f, 1.0f);

	FVector Input = FVector(ClampedForward, ClampedRight, 0.0f);
	if (Input.IsNearlyZero())
	{
		return;
	}
	Input = Input.GetSafeNormal();

	if (!UpdatedComponent)
	{
		return;
	}

	const float TorqueMag = FMath::Max(0.0f, MaxTorque);
	if (TorqueMag <= SMALL_NUMBER)
	{
		return;
	}

	const FVector Up = -GravityDirection.GetSafeNormal();

	FVector Fwd = UpdatedComponent->GetForwardVector();
	FVector Rgt = UpdatedComponent->GetRightVector();

	Fwd = FVector::VectorPlaneProject(Fwd, Up).GetSafeNormal();
	Rgt = FVector::VectorPlaneProject(Rgt, Up).GetSafeNormal();

	if (Fwd.IsNearlyZero() || Rgt.IsNearlyZero())
	{
		return;
	}

	const FVector WishDir = (Fwd * Input.X + Rgt * Input.Y).GetSafeNormal();
	if (WishDir.IsNearlyZero())
	{
		return;
	}

	// Roll axis that produces motion along WishDir on the plane:
	// axis = WishDir x Up
	const FVector RollAxis = FVector::CrossProduct(WishDir, Up).GetSafeNormal();
	if (RollAxis.IsNearlyZero())
	{
		return;
	}

	AddTorque(RollAxis * TorqueMag);
}


void UAG_RigidbodyComponent::AddDriveForwardInput(float ForwardInput, float MaxAccelerationCm, float MaxSpeedCm)
{
	AddDriveInput(ForwardInput, 0.0f, MaxAccelerationCm, MaxSpeedCm);
}

void UAG_RigidbodyComponent::AddDriveRightInput(float RightInput, float MaxAccelerationCm, float MaxSpeedCm)
{
	AddDriveInput(0.0f, RightInput, MaxAccelerationCm, MaxSpeedCm);
}

void UAG_RigidbodyComponent::AddRollTorqueForwardInput(float ForwardInput, float MaxTorque)
{
	AddRollTorqueInput(ForwardInput, 0.0f, MaxTorque);
}

void UAG_RigidbodyComponent::AddRollTorqueRightInput(float RightInput, float MaxTorque)
{
	AddRollTorqueInput(0.0f, RightInput, MaxTorque);
}

void UAG_RigidbodyComponent::ApplyBrake(float Strength, float FixedDeltaTime)
{
	ApplyForwardBrake(Strength, FixedDeltaTime);
	ApplySideBrake(Strength, FixedDeltaTime);
}

void UAG_RigidbodyComponent::ApplySpinBrake(float Strength, float FixedDeltaTime)
{
	ApplyForwardSpinBrake(Strength, FixedDeltaTime);
	ApplySideSpinBrake(Strength, FixedDeltaTime);
}



void UAG_RigidbodyComponent::ApplyForwardBrake(float Strength, float FixedDeltaTime)
{
	if (!UpdatedComponent || Strength <= 0.0f || FixedDeltaTime <= 0.0f)
	{
		return;
	}

	const FVector Up = -GravityDirection; // option A
	FVector Fwd, Right;
	BuildPlanarBasis(UpdatedComponent, Up, Fwd, Right);

	const float Alpha = ComputeBrakeAlpha(Strength, FixedDeltaTime);

	const float VForward = FVector::DotProduct(Velocity, Fwd);
	Velocity -= (VForward * Alpha) * Fwd;
}

void UAG_RigidbodyComponent::ApplySideBrake(float Strength, float FixedDeltaTime)
{
	if (!UpdatedComponent || Strength <= 0.0f || FixedDeltaTime <= 0.0f)
	{
		return;
	}

	const FVector Up = -GravityDirection; // option A
	FVector Fwd, Right;
	BuildPlanarBasis(UpdatedComponent, Up, Fwd, Right);

	const float Alpha = ComputeBrakeAlpha(Strength, FixedDeltaTime);

	const float VSide = FVector::DotProduct(Velocity, Right);
	Velocity -= (VSide * Alpha) * Right;
}

void UAG_RigidbodyComponent::ApplyForwardSpinBrake(float Strength, float FixedDeltaTime)
{
	if (!bEnableRotation || InertiaScalar <= SMALL_NUMBER || !UpdatedComponent || Strength <= 0.0f || FixedDeltaTime <= 0.0f)
	{
		return;
	}

	const FVector Up = -GravityDirection; // option A
	FVector Fwd, Right;
	BuildPlanarBasis(UpdatedComponent, Up, Fwd, Right);

	const float Alpha = ComputeBrakeAlpha(Strength, FixedDeltaTime);

	// Rolling forward corresponds primarily to spin about the Right axis
	const float OmegaAboutRight = FVector::DotProduct(AngularVelocity, Right);
	AngularVelocity -= (OmegaAboutRight * Alpha) * Right;
}

void UAG_RigidbodyComponent::ApplySideSpinBrake(float Strength, float FixedDeltaTime)
{
	if (!bEnableRotation || InertiaScalar <= SMALL_NUMBER || !UpdatedComponent || Strength <= 0.0f || FixedDeltaTime <= 0.0f)
	{
		return;
	}

	const FVector Up = -GravityDirection; // option A
	FVector Fwd, Right;
	BuildPlanarBasis(UpdatedComponent, Up, Fwd, Right);

	const float Alpha = ComputeBrakeAlpha(Strength, FixedDeltaTime);

	// Rolling sideways corresponds primarily to spin about the Forward axis
	const float OmegaAboutForward = FVector::DotProduct(AngularVelocity, Fwd);
	AngularVelocity -= (OmegaAboutForward * Alpha) * Fwd;
}


void UAG_RigidbodyComponent::BuildPlanarBasis(const UPrimitiveComponent* UpdatedComponent, const FVector& Up,
	FVector& OutForward, FVector& OutRight)
{
	// Use component axes, projected onto the plane orthogonal to Up
	const FVector Fwd = UpdatedComponent ? UpdatedComponent->GetForwardVector() : FVector::ForwardVector;
	const FVector Rgt = UpdatedComponent ? UpdatedComponent->GetRightVector()   : FVector::RightVector;

	OutForward = FVector::VectorPlaneProject(Fwd, Up).GetSafeNormal();
	OutRight   = FVector::VectorPlaneProject(Rgt, Up).GetSafeNormal();

	// Fallbacks in degenerate cases
	if (OutForward.IsNearlyZero())
	{
		OutForward = FVector::VectorPlaneProject(FVector::ForwardVector, Up).GetSafeNormal();
	}
	if (OutRight.IsNearlyZero())
	{
		OutRight = FVector::CrossProduct(Up, OutForward).GetSafeNormal();
	}
}

float UAG_RigidbodyComponent::ComputeBrakeAlpha(float Strength, float FixedDeltaTime)
{
	// Strength is a rate-like value (1/s). Clamp so we don't overshoot.
	return FMath::Clamp(Strength * FixedDeltaTime, 0.0f, 1.0f);
}

FVector UAG_RigidbodyComponent::GetPlanarVelocity() const
{
	const FVector Up = -GravityDirection.GetSafeNormal();
	const float vUp = FVector::DotProduct(Velocity, Up);
	return Velocity - vUp * Up;
}

float UAG_RigidbodyComponent::GetPlanarSpeed() const
{
	return GetPlanarVelocity().Size();
}

void UAG_RigidbodyComponent::NudgeOutOfGeometry(float Distance)
{
	if (!UpdatedComponent)
	{
		return;
	}

	const float D = FMath::Max(0.0f, Distance);
	if (D <= SMALL_NUMBER)
	{
		return;
	}

	const FVector Up = -GravityDirection.GetSafeNormal();
	const FVector Delta = Up * D;

	// Try to move out with sweep so we don't tunnel further into something
	FHitResult Hit;
	UpdatedComponent->MoveComponent(Delta, UpdatedComponent->GetComponentQuat(), true, &Hit);

	WakeUp();
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

	// Ensure mass is positive
	if (Mass <= SMALL_NUMBER)
	{
		GEngine->AddOnScreenDebugMessage(
			-1,
			5.0f,
			FColor::Red,
			TEXT("UAG_RigidbodyComponent: Mass is zero or negative! Setting to 1.0f.")
		);
		Mass = 1.0f;
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
		}
	}
	if (UpdatedComponent)
	{
		SetUpdatedComponent(UpdatedComponent);
	}

	if (InertiaScalar <= 0.0f && UpdatedComponent)
	{
		const float Radius = UpdatedComponent->Bounds.SphereRadius; // good enough
		InertiaScalar = 0.4f * Mass * Radius * Radius; // solid sphere approx
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
	if (NumSteps == MaxSubSteps)
	{
		// Prevent spiral of death
		TimeAccumulator = 0.0f;
	}
}


void UAG_RigidbodyComponent::FixedUpdate(float FixedDeltaTime)
{
	// If we can't sleep, ensure we're always awake
	if (!bCanSleep)
	{
		bSleeping = false;
	}
	// Skip simulation if sleeping or massless
	if (bSleeping || Mass <= SMALL_NUMBER)
	{
		return;
	}

	bIsGrounded = false;
	bHadStaticContactThisStep = false;

	IntegrateForces();
	IntegrateVelocity(FixedDeltaTime);
	IntegrateAngularVelocity(FixedDeltaTime);

	DoMovementAndCollisions(FixedDeltaTime);
	SolveGroundContactFriction(FixedDeltaTime);
	IntegrateAngularDrag(FixedDeltaTime);
	
	ClampSpeeds();
	ApplyRotation(FixedDeltaTime);

	UpdateSleepState();

	AccumulatedForces = FVector::ZeroVector;
	AccumulatedTorque = FVector::ZeroVector;
}


void UAG_RigidbodyComponent::IntegrateForces()
{
	ApplyGravity();
	ApplyDragForce();
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

void UAG_RigidbodyComponent::IntegrateAngularDrag(float FixedDeltaTime)
{
	if (!bEnableRotation || AngularDragCoeff <= 0.0f || InertiaScalar <= SMALL_NUMBER)
	{
		return;
	}

	// ω_{n+1} = ω_n / (1 + (c/I) dt)
	const float Denom = 1.0f + (AngularDragCoeff / InertiaScalar) * FixedDeltaTime;
	AngularVelocity /= Denom;
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
			
			break;
		}

		FHitResult Hit;
		const FQuat Rotation = UpdatedComponent->GetComponentQuat();

		// Sweep along full remaining Delta this iteration
		UpdatedComponent->MoveComponent(Delta, Rotation, bEnableCollision, &Hit);

		if (!bEnableCollision || !Hit.bBlockingHit)
		{
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
			break;
		}

		// Slide for the remaining time along the contact plane
		const FVector Normal = Hit.ImpactNormal.GetSafeNormal();
		const float SlideDeltaTime = FixedDeltaTime * RemainingTime;
		const FVector NewDelta = Velocity * SlideDeltaTime;
		const FVector SlideDelta = FVector::VectorPlaneProject(NewDelta, Normal);

		if (SlideDelta.IsNearlyZero())
		{
			// No slide movement, done for this step
			break;
		}

		FHitResult SlideHit;
		UpdatedComponent->MoveComponent(SlideDelta, Rotation, bEnableCollision, &SlideHit);

		if (!bEnableCollision || !SlideHit.bBlockingHit)
		{
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

void UAG_RigidbodyComponent::HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime)
{
	if (!UpdatedComponent)
	{
		return;
	}

	if (TrySolveBodyBody(Hit, FixedDeltaTime))
	{
		return;
	}

	SolveStaticContact(Hit, FixedDeltaTime);
}

void UAG_RigidbodyComponent::UpdateSleepState()
{
	if (!bCanSleep)
	{
		return;
	}

	const float LinearSpeedSq = Velocity.SizeSquared();
	const float AngularSpeedSq = GetContactAngularVelocity().SizeSquared();

	const float SleepLinearSpeedSq = SleepLinearSpeedThreshold * SleepLinearSpeedThreshold;
	const float SleepAngularSpeedSq = SleepAngularSpeedThreshold * SleepAngularSpeedThreshold;

	const bool bBelowLinearThreshold = (LinearSpeedSq < SleepLinearSpeedSq);
	const bool bBelowAngularThreshold = (AngularSpeedSq < SleepAngularSpeedSq);

	if (bIsGrounded && bBelowLinearThreshold && bBelowAngularThreshold)
	{
		++FramesAtRest;

		if (FramesAtRest >= MinFramesAtRest)
		{
			Velocity = FVector::ZeroVector;
			AngularVelocity = FVector::ZeroVector;
			bSleeping = true;
		}
	}
	else
	{
		FramesAtRest = 0;
		bSleeping = false;
	}
}


void UAG_RigidbodyComponent::SolveGroundContactFriction(float FixedDeltaTime)
{
	// Only matters if we want contact friction
	if ((DynamicFrictionCoeff <= 0.0f && StaticFrictionCoeff <= 0.0f) || Mass <= SMALL_NUMBER)
	{
		return;
	}
	
	if (bHadStaticContactThisStep)
	{
		return;
	}

	FHitResult GroundHit;
	if (!TryGetGroundHit(GroundHit) || !GroundHit.bBlockingHit)
	{
		return;
	}

	FAGContactData Contact;
	if (!BuildContactData(GroundHit, Contact))
	{
		return;
	}

	const float UpDot = UpdateGroundStateFromNormal(Contact.Normal);
	if (!bIsGrounded)
	{
		return;
	}

	// IMPORTANT: this is the “resting contact” path:
	// Contact.NormalImpulse will be 0 here, so ApplyFrictionImpulse will fall back to m*g*dt*UpDot.
	ApplyFrictionImpulse(Contact, FixedDeltaTime, UpDot);
	ApplyTorsionalFrictionImpulse(Contact, FixedDeltaTime, UpDot);
	ClampContactNormalRestVelocity(Contact);
}

float UAG_RigidbodyComponent::UpdateGroundStateFromNormal(const FVector& ContactNormal)
{
	const float UpDot = FVector::DotProduct(-GravityDirection, ContactNormal);
	bIsGrounded = (UpDot >= GroundNormalCosThreshold);

	if (bIsGrounded)
	{
		bHasLastGroundNormal = true;
		LastGroundNormal = ContactNormal;

		if (bAdjustGravityToGround)
		{
			AdjustGravityFromContactNormal(false);
			// Recompute after changing gravity
			const float NewUpDot = FVector::DotProduct(-GravityDirection, ContactNormal);
			bIsGrounded = (NewUpDot >= GroundNormalCosThreshold);
			return NewUpDot;
		}
	}

	return UpDot;
}

bool UAG_RigidbodyComponent::TryGetGroundHit(FHitResult& OutHit) const
{
	if (!UpdatedComponent)
	{
		return false;
	}

	const float ProbeDistance = FMath::Max(2.0f, CachedRadius * 0.1f);

	const FVector Start = UpdatedComponent->GetComponentLocation();
	const FVector End   = Start + GravityDirection * ProbeDistance;

	FCollisionQueryParams Params(SCENE_QUERY_STAT(AG_GroundProbe), false, GetOwner());
	return GetWorld()->LineTraceSingleByChannel(OutHit, Start, End, GroundProbeChannel, Params);
}

float UAG_RigidbodyComponent::ComputeTorsionalRadius(const FVector& /*ContactNormal*/) const
{
	//TODO: Replace/extend this for boxes/capsules without touching the solver.
	return CachedRadius * TorsionalRadiusScale;
}

void UAG_RigidbodyComponent::ApplyTorsionalFrictionImpulse(FAGContactData& Contact, float FixedDeltaTime, float UpDot)
{
	if (!bEnableRotation || InertiaScalar <= SMALL_NUMBER)
	{
		return;
	}
	if (TorsionalFrictionCoeff <= 0.0f)
	{
		return;
	}

	const FVector N = Contact.Normal;

	// Spin about the contact normal
	const float OmegaN = FVector::DotProduct(AngularVelocity, N);
	if (FMath::Abs(OmegaN) <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Use the same "effective normal impulse" idea you already use for friction caps.
	float EffectiveNormalImpulse = Contact.NormalImpulse;

	// Resting/grounded contact fallback: m*g*UpDot*dt
	if (EffectiveNormalImpulse <= SMALL_NUMBER && bIsGrounded && UpDot > 0.0f)
	{
		EffectiveNormalImpulse = Mass * GravityStrength * UpDot * FixedDeltaTime;
	}

	if (EffectiveNormalImpulse <= SMALL_NUMBER)
	{
		return;
	}

	const float rEff = ComputeTorsionalRadius(N);
	if (rEff <= SMALL_NUMBER)
	{
		return;
	}

	// Coulomb-like twist cap:
	// tau_max = mu_twist * N_force * r
	// Jw_max  = tau_max * dt = mu_twist * (N_force*dt) * r = mu_twist * NormalImpulse * r
	const float JwMax = TorsionalFrictionCoeff * EffectiveNormalImpulse * rEff;

	// Angular impulse needed to kill OmegaN in one step (scalar inertia simplification)
	float Jw = -OmegaN * InertiaScalar;
	Jw = FMath::Clamp(Jw, -JwMax, JwMax);

	if (FMath::IsNearlyZero(Jw))
	{
		return;
	}

	AngularVelocity += (Jw / InertiaScalar) * N;
}


void UAG_RigidbodyComponent::ApplyRotation(float FixedDeltaTime) const
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
	const FQuat DeltaRot(Axis, Angle);
	UpdatedComponent->AddWorldRotation(DeltaRot, false);
}

FVector UAG_RigidbodyComponent::GetContactAngularVelocity() const
{
	return bEnableRotation ? AngularVelocity : FVector::ZeroVector;
}

bool UAG_RigidbodyComponent::BuildContactData(
	const FHitResult& Hit,
	FAGContactData& OutData
) const
{
	if (!UpdatedComponent)
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

	FVector R = ContactPoint - ComPos;
	
	if (bAssumeSphere && CachedRadius > SMALL_NUMBER)
	{
		// For a sphere, lever arm is along the normal (stable and correct)
		R = -Normal * CachedRadius;
		ContactPoint = ComPos + R; // optional: keep point consistent
	}

	// Contact velocity: v_c = v + ω × r
	const FVector VContact = Velocity + FVector::CrossProduct(GetContactAngularVelocity(), R);

	const float VRelN = FVector::DotProduct(VContact, Normal);

	OutData.Normal = Normal;
	OutData.ContactPoint = ContactPoint;
	OutData.R = R;
	OutData.VContact = VContact;
	OutData.VRelN = VRelN;
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
	const float deltaRelN = desiredRelN - vRelN; // = -(1+e) * vRelN

	// Effective inverse mass along the normal:
	// 1/m + ((r × n)^2 / I) for scalar inertia
	const FVector rCrossN = FVector::CrossProduct(Contact.R, Contact.Normal);
	const float rCrossNLenSq = rCrossN.SizeSquared();

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


void UAG_RigidbodyComponent::ApplyFrictionImpulse(FAGContactData& Contact, float FixedDeltaTime, float UpDot)
{
	// Need at least some friction
	if (DynamicFrictionCoeff <= 0.0f && StaticFrictionCoeff <= 0.0f)
	{
		return;
	}

	// ------------------------------------------------------------
	// 1) Effective normal impulse (Coulomb cap basis)
	// ------------------------------------------------------------
	float EffectiveNormalImpulse = Contact.NormalImpulse;

	// Resting contact support estimate: N ≈ m*g*UpDot
	if (EffectiveNormalImpulse <= SMALL_NUMBER && bIsGrounded && UpDot > 0.0f)
	{
		const float NormalForce = Mass * GravityStrength * UpDot;
		EffectiveNormalImpulse = NormalForce * FixedDeltaTime;
	}

	if (EffectiveNormalImpulse <= SMALL_NUMBER)
	{
		return;
	}

	// ------------------------------------------------------------
	// 2) Tangential velocity at contact
	// ------------------------------------------------------------
	const FVector Omega = GetContactAngularVelocity();
	const FVector VContact = Velocity + FVector::CrossProduct(Omega, Contact.R);

	const float vRelN = FVector::DotProduct(VContact, Contact.Normal);
	const FVector vT = VContact - vRelN * Contact.Normal;
	const float vTLen = vT.Size();

	if (vTLen <= KINDA_SMALL_NUMBER)
	{
		return; // no slip to correct
	}

	const FVector TangentDir = vT / vTLen;

	// ------------------------------------------------------------
	// 3) Effective mass along tangent
	// ------------------------------------------------------------
	const FVector rCrossT = FVector::CrossProduct(Contact.R, TangentDir);
	const float rCrossTLenSq = rCrossT.SizeSquared();

	float invEffMassT = 1.0f / Mass;
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		invEffMassT += rCrossTLenSq / InertiaScalar;
	}

	if (invEffMassT <= SMALL_NUMBER)
	{
		return;
	}

	// ------------------------------------------------------------
	// 4) Static friction first, then dynamic
	// ------------------------------------------------------------
	// Impulse needed to cancel tangential velocity this step:
	// want vT' = 0 => jT_needed = -vTLen / invEffMassT
	const float jTNeeded = -vTLen / invEffMassT;
	const float jTNeededAbs = FMath::Abs(jTNeeded);

	const float maxStatic = StaticFrictionCoeff * EffectiveNormalImpulse;
	const float maxDynamic = DynamicFrictionCoeff * EffectiveNormalImpulse;

	float jT;

	// If static can fully cancel slip, do it (no slip)
	if (StaticFrictionCoeff > 0.0f && jTNeededAbs <= maxStatic)
	{
		jT = jTNeeded;
	}
	else
	{
		// Otherwise dynamic friction clamp (slip)
		if (DynamicFrictionCoeff <= 0.0f)
		{
			return;
		}
		jT = FMath::Clamp(jTNeeded, -maxDynamic, maxDynamic);
	}

	if (FMath::IsNearlyZero(jT))
	{
		return;
	}

	const FVector ImpulseT = jT * TangentDir;

	// Linear
	Velocity += ImpulseT / Mass;

	// Angular (only if rotation enabled)
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		AngularVelocity += FVector::CrossProduct(Contact.R, ImpulseT) / InertiaScalar;
	}
}

void UAG_RigidbodyComponent::ClampContactNormalRestVelocity(FAGContactData& Contact)
{
	// Recompute contact velocity with both impulses applied
	const FVector VContactFinal =
		Velocity + FVector::CrossProduct(GetContactAngularVelocity(), Contact.R);

	const float vRelN_final = FVector::DotProduct(VContactFinal, Contact.Normal);

	const float NormalRestThreshold = 1.0f; // cm/s, tune to taste
	if (FMath::Abs(vRelN_final) < NormalRestThreshold)
	{
		// Approx: remove normal component from linear velocity only
		Velocity -= vRelN_final * Contact.Normal;
	}
}

bool UAG_RigidbodyComponent::TrySolveBodyBody(const FHitResult& Hit, float FixedDeltaTime)
{
	AActor* OtherActor = Hit.GetActor();
	if (!OtherActor)
	{
		return false;
	}

	UAG_RigidbodyComponent* OtherBody = OtherActor->FindComponentByClass<UAG_RigidbodyComponent>();
	if (!OtherBody || OtherBody == this)
	{
		return false;
	}

	OtherBody->WakeUp();
	HandleBodyBodyContact(OtherBody, Hit, FixedDeltaTime);
	return true;
}

void UAG_RigidbodyComponent::SolveStaticContact(const FHitResult& Hit, float FixedDeltaTime)
{
	FAGContactData Contact;
	if (!BuildContactData(Hit, Contact))
	{
		return;
	}
	
	const float UpDot = UpdateGroundStateFromNormal(Contact.Normal);

	// Only treat as "static support contact" if it's ground-ish
	if (UpDot >= GroundNormalCosThreshold)
	{
		bHadStaticContactThisStep = true;
	}

	const float NormalImpactThreshold = 1.0f;
	if (Contact.VRelN < -NormalImpactThreshold)
	{
		ApplyNormalImpulse(Contact);
	}

	ApplyFrictionImpulse(Contact, FixedDeltaTime, UpDot);
	ApplyTorsionalFrictionImpulse(Contact, FixedDeltaTime, UpDot);
	ClampContactNormalRestVelocity(Contact);
}

void UAG_RigidbodyComponent::HandleBodyBodyContact(UAG_RigidbodyComponent* OtherBody, const FHitResult& Hit,
                                                   float FixedDeltaTime)
{
	FAGTwoBodyContactData Contact;
	if (!BuildBodyBodyContactData(OtherBody, Hit, Contact))
	{
		return;
	}
	
	const float Jn = ApplyTwoBodyNormalImpulse(OtherBody, Contact);
	const float JnAbs = FMath::Abs(Jn);

	ApplyTwoBodyFrictionImpulse(OtherBody, Contact, JnAbs);
	ApplyTwoBodyTorsionalFrictionImpulse(OtherBody, Contact, JnAbs);
}

bool UAG_RigidbodyComponent::BuildBodyBodyContactData(UAG_RigidbodyComponent* OtherBody, const FHitResult& Hit, FAGTwoBodyContactData&  OutData) const
{
	if (!OtherBody || !UpdatedComponent || !OtherBody->UpdatedComponent)
	{
		return false;
	}

	// Contact normal from other → this (ImpactNormal has that convention)
	const FVector Normal = Hit.ImpactNormal.GetSafeNormal();
	if (Normal.IsNearlyZero())
	{
		return false;
	}

	// Contact point
	FVector ContactPoint = Hit.ImpactPoint;
	if (ContactPoint.IsNearlyZero())
	{
		ContactPoint = UpdatedComponent->GetComponentLocation();
	}

	// Centers of mass (we assume COM at component origin)
	const FVector X1 = UpdatedComponent->GetComponentLocation();
	const FVector X2 = OtherBody->UpdatedComponent->GetComponentLocation();

	FVector R1 = ContactPoint - X1;
	FVector R2 = ContactPoint - X2;
	
	
	if (bAssumeSphere && CachedRadius > SMALL_NUMBER)
	{
		R1 = -Normal * CachedRadius;
	}
	if (OtherBody->bAssumeSphere && OtherBody->CachedRadius > SMALL_NUMBER)
	{
		R2 =  Normal * OtherBody->CachedRadius;
	}

	// Velocities at the contact point
	const FVector V1c = Velocity + FVector::CrossProduct(GetContactAngularVelocity(), R1);
	const FVector V2c = OtherBody->Velocity + FVector::CrossProduct(OtherBody->GetContactAngularVelocity(), R2);

	const FVector VRel  = V1c - V2c;
	const float   VRelN = FVector::DotProduct(VRel, Normal);

	OutData.Normal       = Normal;
	OutData.ContactPoint = ContactPoint;
	OutData.R1           = R1;
	OutData.R2           = R2;
	OutData.VRel         = VRel;
	OutData.VRelN        = VRelN;

	return true;
}


float UAG_RigidbodyComponent::ApplyTwoBodyNormalImpulse(UAG_RigidbodyComponent* OtherBody, FAGTwoBodyContactData&  Contact)
{
	if (!OtherBody)
	{
		return 0.0f;
	}

	if (Mass <= SMALL_NUMBER || OtherBody->Mass <= SMALL_NUMBER)
	{
		return 0.0f; // treat massless as static / ignore for now
	}

	const FVector N = Contact.Normal;

	// Combined restitution 
	const float E1 = FMath::Max(Restitution,           0.0f);
	const float E2 = FMath::Max(OtherBody->Restitution, 0.0f);
	const float E  = 0.5f * (E1 + E2);

	const float VRelN = Contact.VRelN;
	
	if (VRelN >= 0.0f)
	{
		return 0.0f; // bodies separating or stationary along normal
	}

	// Effective inverse mass along normal:
	// 1/m1 + 1/m2 + (|r1×n|^2 / I1) + (|r2×n|^2 / I2)
	const FVector R1xN = FVector::CrossProduct(Contact.R1, N);
	const FVector R2xN = FVector::CrossProduct(Contact.R2, N);

	float InvEffMassN = 1.0f / Mass + 1.0f / OtherBody->Mass;

	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		InvEffMassN += R1xN.SizeSquared() / InertiaScalar;
	}
	if (OtherBody->bEnableRotation && OtherBody->InertiaScalar > 0.0f)
	{
		InvEffMassN += R2xN.SizeSquared() / OtherBody->InertiaScalar;
	}

	if (InvEffMassN <= SMALL_NUMBER)
	{
		return 0.0f;
	}

	// Desired post-impact normal relative velocity
	const float DesiredRelN = -E * VRelN;
	const float DeltaRelN   = DesiredRelN - VRelN; // = -(1+E) * VRelN

	float Jn = DeltaRelN / InvEffMassN;
	if (Jn < 0.0f) { Jn = 0.0f; }

	const FVector ImpulseN = Jn * N;

	// Apply to this body
	Velocity += ImpulseN / Mass;
	if (bEnableRotation && InertiaScalar > 0.0f)
	{
		AngularVelocity += FVector::CrossProduct(Contact.R1, ImpulseN) / InertiaScalar;
	}

	// Apply opposite impulse to other body
	OtherBody->Velocity -= ImpulseN / OtherBody->Mass;
	if (OtherBody->bEnableRotation && OtherBody->InertiaScalar > 0.0f)
	{
		const FVector OppImpulseN = -ImpulseN;
		OtherBody->AngularVelocity += FVector::CrossProduct(Contact.R2, OppImpulseN) / OtherBody->InertiaScalar;
	}
	
	return Jn;
}

void UAG_RigidbodyComponent::ApplyTwoBodyFrictionImpulse(
	UAG_RigidbodyComponent* OtherBody,
	const FAGTwoBodyContactData& Contact,
	float NormalImpulseMagnitude
)
{
	if (!OtherBody)
	{
		return;
	}

	// No normal support -> no Coulomb friction cap
	if (NormalImpulseMagnitude <= SMALL_NUMBER)
	{
		return;
	}

	// Combine coefficients (simple, stable default)
	const float MuStatic  = 0.5f * (StaticFrictionCoeff  + OtherBody->StaticFrictionCoeff);
	const float MuDynamic = 0.5f * (DynamicFrictionCoeff + OtherBody->DynamicFrictionCoeff);

	if (MuStatic <= 0.0f && MuDynamic <= 0.0f)
	{
		return;
	}

	const FVector N = Contact.Normal;

	// Relative velocity at contact (already computed in BuildBodyBodyContactData)
	const FVector VRel = Contact.VRel;

	// Tangential relative velocity
	const float VRelN = FVector::DotProduct(VRel, N);
	const FVector VT = VRel - VRelN * N;
	const float VTLen = VT.Size();

	if (VTLen <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector T = VT / VTLen;

	// Effective inverse mass along tangent for two bodies:
	// 1/m1 + 1/m2 + |r1×t|^2/I1 + |r2×t|^2/I2
	float InvEffMassT = 1.0f / Mass + 1.0f / OtherBody->Mass;

	if (bEnableRotation && InertiaScalar > SMALL_NUMBER)
	{
		InvEffMassT += FVector::CrossProduct(Contact.R1, T).SizeSquared() / InertiaScalar;
	}
	if (OtherBody->bEnableRotation && OtherBody->InertiaScalar > SMALL_NUMBER)
	{
		InvEffMassT += FVector::CrossProduct(Contact.R2, T).SizeSquared() / OtherBody->InertiaScalar;
	}

	if (InvEffMassT <= SMALL_NUMBER)
	{
		return;
	}

	// Impulse needed to kill VT in one shot
	const float JTNeeded = -VTLen / InvEffMassT;
	const float JTNeededAbs = FMath::Abs(JTNeeded);

	const float MaxStatic  = MuStatic  * NormalImpulseMagnitude;
	const float MaxDynamic = MuDynamic * NormalImpulseMagnitude;

	float JT = 0.0f;

	// Static if it can fully cancel slip, otherwise dynamic
	if (MuStatic > 0.0f && JTNeededAbs <= MaxStatic)
	{
		JT = JTNeeded;
	}
	else
	{
		if (MuDynamic <= 0.0f)
		{
			return;
		}
		JT = FMath::Clamp(JTNeeded, -MaxDynamic, MaxDynamic);
	}

	if (FMath::IsNearlyZero(JT))
	{
		return;
	}

	const FVector ImpulseT = JT * T;

	// Apply linear impulses (equal and opposite)
	Velocity += ImpulseT / Mass;
	OtherBody->Velocity -= ImpulseT / OtherBody->Mass;

	// Apply angular impulses
	if (bEnableRotation && InertiaScalar > SMALL_NUMBER)
	{
		AngularVelocity += FVector::CrossProduct(Contact.R1, ImpulseT) / InertiaScalar;
	}
	if (OtherBody->bEnableRotation && OtherBody->InertiaScalar > SMALL_NUMBER)
	{
		OtherBody->AngularVelocity -= FVector::CrossProduct(Contact.R2, ImpulseT) / OtherBody->InertiaScalar;
	}
}


void UAG_RigidbodyComponent::ApplyTwoBodyTorsionalFrictionImpulse(
	UAG_RigidbodyComponent* OtherBody,
	const FAGTwoBodyContactData& Contact,
	float NormalImpulseMagnitude
)
{
	if (!OtherBody)
	{
		return;
	}
	if (NormalImpulseMagnitude <= SMALL_NUMBER)
	{
		return;
	}

	// Need at least one rotating body to matter
	const bool bThisRot  = bEnableRotation && InertiaScalar > SMALL_NUMBER;
	const bool bOtherRot = OtherBody->bEnableRotation && OtherBody->InertiaScalar > SMALL_NUMBER;
	if (!bThisRot && !bOtherRot)
	{
		return;
	}

	// Combine torsional coeff
	const float MuTwist = 0.5f * (TorsionalFrictionCoeff + OtherBody->TorsionalFrictionCoeff);
	if (MuTwist <= 0.0f)
	{
		return;
	}

	const FVector N = Contact.Normal;

	const float Omega1N = bThisRot  ? FVector::DotProduct(AngularVelocity, N) : 0.0f;
	const float Omega2N = bOtherRot ? FVector::DotProduct(OtherBody->AngularVelocity, N) : 0.0f;

	const float OmegaRelN = Omega1N - Omega2N; // relative twist rate about N
	if (FMath::Abs(OmegaRelN) <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	// Effective torsional radius: average of each body's notion (sphere now, extensible later)
	const float R1 = ComputeTorsionalRadius(N);
	const float R2 = OtherBody->ComputeTorsionalRadius(-N);
	const float REff = 0.5f * (R1 + R2);

	if (REff <= SMALL_NUMBER)
	{
		return;
	}

	// Cap angular impulse (Coulomb-like): Jw_max = mu_twist * Jn * r
	const float JwMax = MuTwist * NormalImpulseMagnitude * REff;

	// Relative change in OmegaRelN from an angular impulse Jw along N:
	// Omega1 += Jw/I1, Omega2 -= Jw/I2  -> OmegaRel += Jw*(1/I1 + 1/I2)
	float InvEffInertia = 0.0f;
	if (bThisRot)  { InvEffInertia += 1.0f / InertiaScalar; }
	if (bOtherRot) { InvEffInertia += 1.0f / OtherBody->InertiaScalar; }

	if (InvEffInertia <= SMALL_NUMBER)
	{
		return;
	}

	float JwNeeded = -OmegaRelN / InvEffInertia;
	float Jw = FMath::Clamp(JwNeeded, -JwMax, JwMax);

	if (FMath::IsNearlyZero(Jw))
	{
		return;
	}

	// Apply equal/opposite angular impulse along N
	if (bThisRot)
	{
		AngularVelocity += (Jw / InertiaScalar) * N;
	}
	if (bOtherRot)
	{
		OtherBody->AngularVelocity -= (Jw / OtherBody->InertiaScalar) * N;
	}
}

void UAG_RigidbodyComponent::ClampSpeeds()
{
	// ----------------------------
	// 1) Linear velocity
	// ----------------------------

	if (MaxPlanarSpeed > 0.0f)
	{
		// Define "up"
		const FVector Up = /*
			(bIsGrounded && bHasLastGroundNormal)
			? LastGroundNormal
			:*/ -GravityDirection;

		// Decompose velocity
		const float Vn = FVector::DotProduct(Velocity, Up);
		const FVector VNormal = Vn * Up;
		FVector VPlanar = Velocity - VNormal;

		const float PlanarSpeed = VPlanar.Size();
		if (PlanarSpeed > MaxPlanarSpeed)
		{
			VPlanar *= (MaxPlanarSpeed / PlanarSpeed);
			Velocity = VPlanar + VNormal;
		}
	}

	// ----------------------------
	// 2) Fall speed
	// ----------------------------

	if (MaxFallSpeed > 0.0f)
	{
		// GravityDirection points *down*
		const float Vg = FVector::DotProduct(Velocity, GravityDirection);

		if (Vg > MaxFallSpeed)
		{
			Velocity -= (Vg - MaxFallSpeed) * GravityDirection;
		}
	}

	// ----------------------------
	// 3) Angular velocity
	// ----------------------------

	if (bEnableRotation && MaxAngularSpeed > 0.0f)
	{
		const float Omega = AngularVelocity.Size();
		if (Omega > MaxAngularSpeed)
		{
			AngularVelocity *= (MaxAngularSpeed / Omega);
		}
	}
}


