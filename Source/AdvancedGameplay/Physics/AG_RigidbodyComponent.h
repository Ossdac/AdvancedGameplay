#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AG_RigidbodyComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ADVANCEDGAMEPLAY_API UAG_RigidbodyComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAG_RigidbodyComponent();

protected:
	virtual void BeginPlay() override;
	
	// Fixed-step "physics" update
	UFUNCTION()
	void FixedUpdate(float FixedDeltaTime);

	// Force contribution functions
	UFUNCTION()
	void ApplyGravity();
	UFUNCTION()
	void ApplyDragForce();

	// Collision response
	UFUNCTION()
	void HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime);
	
	UFUNCTION(Blueprintable)
	void SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent);
	
	UFUNCTION()
	void UpdateSleepState();

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

	// Current velocity in cm/s
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Runtime")
	FVector Velocity = FVector(0.0f, 0.0f, .0f);
	
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	UPrimitiveComponent* UpdatedComponent = nullptr;

	// Enable or disable collision handling
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	bool bEnableCollision = true;

	// 0 = no bounce, 1 = perfectly elastic
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	float Restitution = 0.0f;

	// Coefficients for a simple friction model
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float StaticFrictionSpeedThreshold  = 0.5f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float DynamicFrictionCoeff  = 0.3f;
	
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Solver")
	int32 MaxContactIterations = 4; 
	
	// Linear "viscous" drag: F_drag_linear = -LinearDragCoeff * v
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Damping")
	float LinearDragCoeff = 0.0f;

	// Quadratic drag: F_drag_quad = -QuadraticDragCoeff * |v| * v
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Damping")
	float QuadraticDragCoeff = 0.0f;
	
	// --- Sleeping / grounded state ---

	// Are we currently treated as resting on a "ground-like" surface this step?
	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Sleeping")
	bool bIsGrounded = false;

	// Are we currently sleeping (simulation skipped)?
	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Sleeping")
	bool bSleeping = false;

	// Cosine threshold for treating a contact normal as "ground" relative to gravity.
	// Dot(-GravityDirection, Normal) >= GroundNormalCosThreshold → considered ground.
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	float GroundNormalCosThreshold = 0.6f; // ~> max ~53° slope

	// Below this linear speed (cm/s) while grounded, we consider the body "at rest".
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	float SleepLinearSpeedThreshold = 1.0f;

	// Number of consecutive frames at rest before actually going to sleep.
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	int32 MinFramesAtRest = 5;

	int32 FramesAtRest = 0;
	
	// Accumulated force for this step (N in “Unreal units”)
	FVector AccumulatedForces = FVector::ZeroVector;

private:
	float TimeAccumulator = 0.0f;

public:
	virtual void TickComponent(
		float DeltaTime,
		enum ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;
	void IntegrateForces();
	void IntegrateVelocity(float FixedDeltaTime);
	void DoMovementAndCollisions(float FixedDeltaTime);

	// API for external systems to push forces
	void AddForce(const FVector& Force);
};
