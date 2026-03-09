// ProceduralSpiderGaitComponent.h

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpiderGaitTypes.h"
#include "ProceduralSpiderGaitComponent.generated.h"

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ADVANCEDGAMEPLAY_API UProceduralSpiderGaitComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UProceduralSpiderGaitComponent();

	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UFUNCTION(BlueprintCallable, BlueprintPure)
	bool GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const;

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float DutyFactor = 0.68f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepLength1 = 600.0f; // front legs: L1 / R1

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepLength2 = 750.0f; // second row: L2 / R2

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepLength3 = 900.0f; // third row: L3 / R3

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepLength4 = 800.0f; // rear row: L4 / R4

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepHeight1 = 35.0f; // front legs

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepHeight2 = 45.0f; // second row

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepHeight3 = 50.0f; // third row

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepHeight4 = 40.0f; // rear row

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepTriggerDistance = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float TraceUp = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float TraceDown = 80.0f;

	// Only snap a foot to the ground if the hit is within this distance of the current desired point.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float MaxSnapToGround = 150.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	TEnumAsByte<ECollisionChannel> GroundTraceChannel = ECC_WorldStatic;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Rig")
	TArray<FSpiderLegRuntime> Legs;

	// Debug: draw target spheres.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Debug")
	bool bDrawFootTargets = false;
	
	bool IsInitialized() const { return bInitializedFromPose; }
	
	// Controller drives this (units/sec)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float CommandedSpeed = 0.f;

	

	// Derived cycle duration
	float CurrentCycleSeconds = 1.f;

	// active stepping group
	bool bGroupASwings = true;

	// number of legs that still must land in current group
	int32 RemainingInGroup = 0;
	


private:
	bool bInitializedFromPose = false;
	FTimerHandle InitTimerHandle;

	void InitializeFromCurrentPose();
	
private:
	void InitializeDefaultsIfEmpty();
	void UpdateCycle(float DeltaTime);
	void UpdateLeg(FSpiderLegRuntime& Leg, float DeltaTime, float Cycle01);

	bool IsLegInGroupA(ESpiderLeg LegId) const;

	bool SampleGround(const FVector& WorldFrom, FVector& OutHitPoint, FVector& OutHitNormal) const;
	FVector ComputeDesiredFootPoint(const FSpiderLegRuntime& Leg) const;

	void BeginStep(FSpiderLegRuntime& Leg, const FVector& NewEndWorld);
	void TickStep(FSpiderLegRuntime& Leg, float DeltaTime, float SwingDuration);
	
	float GetStepLengthForLeg(ESpiderLeg LegId) const;
	float GetStepHeightForLeg(ESpiderLeg LegId) const;
};