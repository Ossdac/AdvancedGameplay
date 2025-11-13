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
	void FixedUpdate(float FixedDeltaTime);

	// Force contribution functions
	void ApplyGravity(float FixedDeltaTime);

	// Fixed-step config
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Fixed Step")
	float FixedTimeStep = 1.0f / 60.0f;

	// Mass (kg-ish, but in Unreal units it's really arbitrary; just be consistent)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Variables")
	float Mass = 1.0f;

	// Gravity strength in cm/s^2 (980 ~= 1g in Unreal units)
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Variables")
	float GravityStrength = 980.0f;

	// Normalized gravity direction
	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Variables")
	FVector GravityDirection = FVector(0.0f, 0.0f, -1.0f);

	// Current velocity in cm/s
	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	FVector Velocity = FVector::ZeroVector;

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

	// API for external systems to push forces
	void AddForce(const FVector& Force);
};
