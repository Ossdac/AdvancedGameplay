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

	// Read by AnimInstance
	UFUNCTION(BlueprintCallable, BlueprintPure)
	bool GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const;

public:
	// --- Gait tuning (based on paper ranges) ---
	// Duty factor > 0.6 means long stance time.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float DutyFactor = 0.68f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float CycleSeconds = 1.6f; // typical gait cycle ~1.55–1.77s in the referenced study

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepHeight = 6.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepTriggerDistance = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float StepReachForward = 12.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float TraceUp = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	float TraceDown = 80.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Gait")
	TEnumAsByte<ECollisionChannel> GroundTraceChannel = ECC_WorldStatic;

	// 8 legs
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Rig")
	TArray<FSpiderLegRuntime> Legs;

private:
	float AccumTime = 0.0f;
	
	int32 PrevHalf = -1;

	// group toggle each half-cycle
	bool bGroupASwings = true;

private:
	void InitializeDefaultsIfEmpty();
	void UpdateCycle(float DeltaTime);
	void UpdateLeg(FSpiderLegRuntime& Leg, float DeltaTime, float Cycle01);

	bool IsLegInGroupA(ESpiderLeg LegId) const;
	bool IsLegAllowedToSwing(const FSpiderLegRuntime& Leg, float Cycle01) const;

	bool SampleGround(const FVector& WorldFrom, FVector& OutHitPoint, FVector& OutHitNormal) const;
	FVector ComputeDesiredFootPoint(const FSpiderLegRuntime& Leg) const;

	void BeginStep(FSpiderLegRuntime& Leg, const FVector& NewEndWorld);
	void TickStep(FSpiderLegRuntime& Leg, float DeltaTime, float SwingDuration);
};