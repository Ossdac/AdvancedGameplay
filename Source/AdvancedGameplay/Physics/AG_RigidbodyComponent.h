#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "AG_RigidbodyComponent.generated.h"

class UPrimitiveComponent;

USTRUCT(BlueprintType)
struct FAGContactData
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere)
	FVector Normal = FVector::ZeroVector;          // unit

	UPROPERTY(VisibleAnywhere)
	FVector ContactPoint = FVector::ZeroVector;    // world

	UPROPERTY(VisibleAnywhere)
	FVector R = FVector::ZeroVector;               // ContactPoint - COM

	UPROPERTY(VisibleAnywhere)
	FVector VContact = FVector::ZeroVector;        // v + ω × r

	UPROPERTY(VisibleAnywhere)
	float VRelN = 0.0f;                            // VContact · Normal

	UPROPERTY(VisibleAnywhere)
	float NormalImpulse = 0.0f;                    // cached for Coulomb cap
};

USTRUCT(BlueprintType)
struct FAGTwoBodyContactData
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere)
	FVector Normal = FVector::ZeroVector;          // unit

	UPROPERTY(VisibleAnywhere)
	FVector ContactPoint = FVector::ZeroVector;    // world

	UPROPERTY(VisibleAnywhere)
	FVector R1 = FVector::ZeroVector;              // from body 1 COM to contact

	UPROPERTY(VisibleAnywhere)
	FVector R2 = FVector::ZeroVector;              // from body 2 COM to contact

	UPROPERTY(VisibleAnywhere)
	FVector VRel = FVector::ZeroVector;            // V1c - V2c at contact

	UPROPERTY(VisibleAnywhere)
	float VRelN = 0.0f;                            // dot(VRel, Normal)
};

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ADVANCEDGAMEPLAY_API UAG_RigidbodyComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UAG_RigidbodyComponent();

	// --- UActorComponent ---
	virtual void BeginPlay() override;
	virtual void TickComponent(
		float DeltaTime,
		enum ELevelTick TickType,
		FActorComponentTickFunction* ThisTickFunction
	) override;

	// --- Public API ---
	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Setup")
	void SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent);

	// GravityDirection is normalized internally.
	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Forces")
	void SetGravityDirection(
		const FVector& NewGravityDirection,
		float NewGravityStrength = 980.0f,
		bool bWakeUp = true
	);

	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Angular")
	void SetRotationEnabled(bool bEnable, bool bClearSpin = false);

	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Angular")
	void ClearSpin();

	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Forces")
	void AdjustGravityFromContactNormal(bool bWakeUp = false);

	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Forces")
	void AddForce(const FVector& Force);

	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Forces")
	void AddTorque(const FVector& Torque);

	UFUNCTION(BlueprintCallable, Category="AG Rigidbody|Sleep")
	void WakeUp();

protected:
	// --- Fixed-step simulation ---
	void FixedUpdate(float FixedDeltaTime);

	// Integration pipeline
	void IntegrateForces();
	void IntegrateVelocity(float FixedDeltaTime);
	void IntegrateAngularVelocity(float FixedDeltaTime);
	void IntegrateAngularDrag(float FixedDeltaTime);
	void DoMovementAndCollisions(float FixedDeltaTime);

	// Force contributions
	void ApplyGravity();
	void ApplyDragForce();
	

	// Collision response + sleeping
	void HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime);
	void UpdateSleepState();

	// -----------------------------
	// Configuration / Tunables
	// -----------------------------

	// Fixed-step config
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Fixed Step")
	float FixedTimeStep = 1.0f / 60.0f;

	// Solver parameters
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Solver", meta=(ClampMin="1"))
	int32 MaxContactIterations = 4;

	// Shape assumptions / cached geometry
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Shape")
	bool bAssumeSphere = true;

	// Core physical properties
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Properties", meta=(ClampMin="0.0001"))
	float Mass = 1.0f;

	// Gravity strength in cm/s^2 (980 ~= 1g)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Gravity", meta=(ClampMin="0.0"))
	float GravityStrength = 980.0f;

	// Normalized gravity direction
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Gravity")
	FVector GravityDirection = FVector(0.0f, 0.0f, -1.0f);

	// If true, when grounded we align gravity to ground normal
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Gravity")
	bool bAdjustGravityToGround = false;

	// Collision
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	bool bEnableCollision = true;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision")
	TEnumAsByte<ECollisionChannel> GroundProbeChannel = ECC_Visibility;

	// 0 = no bounce, 1 = perfectly elastic
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Collision", meta=(ClampMin="0.0"))
	float Restitution = 0.0f;

	// Friction
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction", meta=(ClampMin="0.0"))
	float StaticFrictionSpeedThreshold = 0.5f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction", meta=(ClampMin="0.0"))
	float StaticFrictionCoeff = 0.8f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction", meta=(ClampMin="0.0"))
	float DynamicFrictionCoeff = 0.3f;
	
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float TorsionalFrictionCoeff = 0.05f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float TorsionalRadiusScale = 1.0f;

	// Linear drag
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Damping", meta=(ClampMin="0.0"))
	float LinearDragCoeff = 0.0f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Damping", meta=(ClampMin="0.0"))
	float QuadraticDragCoeff = 0.0f;

	// Angular dynamics
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Angular")
	bool bEnableRotation = true;

		// When zero it recalculates as a solid sphere
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Angular", meta=(ClampMin="0.0"))
	float InertiaScalar = 0.0f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Angular", meta=(ClampMin="0.0"))
	float AngularDragCoeff = 2000.0f;

	// Sleeping / grounded classification
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping")
	bool bCanSleep = true;

	// Dot(-GravityDirection, Normal) >= threshold => ground
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping", meta=(ClampMin="-1.0", ClampMax="2.0"))
	float GroundNormalCosThreshold = 0.6f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping", meta=(ClampMin="0.0"))
	float SleepLinearSpeedThreshold = 1.0f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping", meta=(ClampMin="0.0"))
	float SleepAngularSpeedThreshold = 0.1f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Sleeping", meta=(ClampMin="1"))
	int32 MinFramesAtRest = 5;

	// -----------------------------
	// Runtime State (not editable)
	// -----------------------------

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	UPrimitiveComponent* UpdatedComponent = nullptr;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Runtime")
	FVector Velocity = FVector::ZeroVector;              // cm/s

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Runtime")
	FVector AngularVelocity = FVector::ZeroVector;       // rad/s

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	bool bIsGrounded = false;

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	bool bHasLastGroundNormal = false;

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	FVector LastGroundNormal = FVector(0.0f, 0.0f, 1.0f);

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	bool bSleeping = false;

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	int32 FramesAtRest = 0;

	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	float CachedRadius = 0.0f;

	// Per-step accumulators
	FVector AccumulatedForces = FVector::ZeroVector;
	FVector AccumulatedTorque = FVector::ZeroVector;

private:
	// Step-local flags
	bool bHadStaticContactThisStep = false;

	// Ground / contact helpers
	void SolveGroundContactFriction(float FixedDeltaTime);
	float UpdateGroundStateFromNormal(const FVector& ContactNormal);
	bool TryGetGroundHit(FHitResult& OutHit) const;
	float ComputeTorsionalRadius(const FVector& ContactNormal) const;
	void ApplyTorsionalFrictionImpulse(FAGContactData& Contact, float FixedDeltaTime, float UpDot);


	// Rotation application (post-collision)
	void ApplyRotation(float FixedDeltaTime) const;

	// Static contact helpers
	bool BuildContactData(const FHitResult& Hit, FAGContactData& OutData) const;
	void ApplyNormalImpulse(FAGContactData& Contact);
	void ApplyFrictionImpulse(FAGContactData& Contact, float FixedDeltaTime, float UpDot);
	void ClampContactNormalRestVelocity(FAGContactData& Contact);

	// Body-body contact helpers
	bool TrySolveBodyBody(const FHitResult& Hit, float FixedDeltaTime);
	void SolveStaticContact(const FHitResult& Hit, float FixedDeltaTime);

	void HandleBodyBodyContact(UAG_RigidbodyComponent* OtherBody, const FHitResult& Hit, float FixedDeltaTime);
	bool BuildBodyBodyContactData(UAG_RigidbodyComponent* OtherBody, const FHitResult& Hit, FAGTwoBodyContactData& OutData) const;
	float ApplyTwoBodyNormalImpulse(UAG_RigidbodyComponent* OtherBody, FAGTwoBodyContactData& Contact);
	
	void ApplyTwoBodyFrictionImpulse(
	UAG_RigidbodyComponent* OtherBody,
	const FAGTwoBodyContactData& Contact,
	float NormalImpulseMagnitude
);

	void ApplyTwoBodyTorsionalFrictionImpulse(
		UAG_RigidbodyComponent* OtherBody,
		const FAGTwoBodyContactData& Contact,
		float NormalImpulseMagnitude
	);

	FORCEINLINE FVector GetContactAngularVelocity() const
	{
		return bEnableRotation ? AngularVelocity : FVector::ZeroVector;
	}

	// Accumulated time since last fixed-step tick
	float TimeAccumulator = 0.0f;
};
