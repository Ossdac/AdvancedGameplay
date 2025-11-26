#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AG_RigidbodyComponent.generated.h"

class UPrimitiveComponent;

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ADVANCEDGAMEPLAY_API UAG_RigidbodyComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAG_RigidbodyComponent();

	// Main per-frame entry point
	virtual void TickComponent(
		float DeltaTime,
		enum ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;

	// External API: forces / torques / component binding
	UFUNCTION(BlueprintCallable, Category="AG Rigidbody")
	void SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent);

	// API for external systems to push forces and torques
	void AddForce(const FVector& Force);
	void AddTorque(const FVector& Torque);

protected:
	// Lifecycle
	virtual void BeginPlay() override;

	// Fixed-step "physics" update
	void FixedUpdate(float FixedDeltaTime);

	// Integration pipeline
	void IntegrateForces();                  
	void IntegrateVelocity(float FixedDeltaTime);
	void IntegrateAngularVelocity(float FixedDeltaTime);
	void DoMovementAndCollisions(float FixedDeltaTime);

	// Force contribution functions
	void ApplyGravity();
	void ApplyDragForce();
	void ApplyAngularDrag();

	// Collision response + sleeping
	void HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime);
	void UpdateSleepState();

	// Optionally clamp tiny angular motion when resting (not yet used)
	void ApplyAngularSleepClamp(); // stub / future use

	// ------------------------------------------------------------------------------------
	// CONFIGURATION / TUNABLES
	// ------------------------------------------------------------------------------------

	// Fixed-step config
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Fixed Step")
	float FixedTimeStep = 1.0f / 60.0f;

	// Mass
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Variables")
	float Mass = 1.0f;

	// Gravity strength in cm/s^2 (980 ~= 1g in Unreal units)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Variables")
	float GravityStrength = 980.0f;

	// Normalized gravity direction
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Variables")
	FVector GravityDirection = FVector(0.0f, 0.0f, -1.0f);

	// Component we drive
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	UPrimitiveComponent* UpdatedComponent = nullptr;

	// Enable or disable collision handling
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	bool bEnableCollision = true;

	// 0 = no bounce, 1 = perfectly elastic
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	float Restitution = 0.0f;

	// Friction model (velocity-based)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float StaticFrictionSpeedThreshold = 0.5f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float DynamicFrictionCoeff = 0.3f;

	// Solver parameters
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Solver")
	int32 MaxContactIterations = 4;

	// Linear damping / drag
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Damping")
	float LinearDragCoeff = 0.0f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Damping")
	float QuadraticDragCoeff = 0.0f;

	// --- Rotation / angular dynamics ---

	// Enable simple angular motion
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Angular")
	bool bEnableRotation = true;

	// Scalar moment of inertia about any axis (for spheres: 0.4 * m * r^2)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Angular")
	float InertiaScalar = 1.0f;

	// Simple angular damping (like linear drag but for spin)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Angular")
	float AngularDragCoeff = 0.0f;

	// --- Sleeping / grounded state ---

	// Enable sleeping behavior
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	bool bCanSleep = true;
	// Cosine threshold for treating a contact normal as "ground" relative to gravity.
	// Dot(-GravityDirection, Normal) >= GroundNormalCosThreshold → considered ground.
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	float GroundNormalCosThreshold = 0.6f;

	// Below this linear speed (cm/s) while grounded, we consider the body "at rest".
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	float SleepLinearSpeedThreshold = 1.0f;

	// Number of consecutive frames at rest before actually going to sleep.
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	int32 MinFramesAtRest = 5;

	// ------------------------------------------------------------------------------------
	// RUNTIME STATE
	// ------------------------------------------------------------------------------------

	// Current linear velocity in cm/s
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Runtime")
	FVector Velocity = FVector::ZeroVector;

	// World-space angular velocity [rad/s]
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Runtime")
	FVector AngularVelocity = FVector::ZeroVector;

	// Is this body currently resting on something "ground-like"?
	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	bool bIsGrounded = false;

	// Are we currently sleeping (simulation skipped)?
	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	bool bSleeping = false;

	// Frames we have been within sleep thresholds while grounded
	int32 FramesAtRest = 0;

	// Accumulated force for this step
	FVector AccumulatedForces = FVector::ZeroVector;

	// Accumulated torque for this step
	FVector AccumulatedTorque = FVector::ZeroVector;

private:
	// Accumulated time since last fixed-step tick
	float TimeAccumulator = 0.0f;
};
