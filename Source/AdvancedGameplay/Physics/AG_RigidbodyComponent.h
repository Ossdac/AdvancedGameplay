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
	
	// Collision response
	void HandleBlockingHit(const FHitResult& Hit, float FixedDeltaTime);
	
	UFUNCTION(Blueprintable)
	void SetUpdatedComponent(UPrimitiveComponent* NewUpdatedComponent);

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
	UPROPERTY(VisibleAnywhere, Category="AG Rigidbody|Runtime")
	FVector Velocity = FVector(0.0f, 100.0f, .0f);
	
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
	float StaticFriction = 0.5f;

	UPROPERTY(EditAnywhere, Category="AG Rigidbody|Friction")
	float DynamicFriction = 0.3f;

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
