#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "AG_SpiderTypes.h"
#include "AG_SpiderAnimInstance.generated.h"

UCLASS()
class ADVANCEDGAMEPLAY_API UAG_SpiderAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

public:
	virtual void NativeInitializeAnimation() override;
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;

	// Read by anim node (thread-safe copy via proxy).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Config")
	TArray<FAG_LegBones> Legs; // size 8

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Config")
	TArray<FVector> NominalFootCS; // size 8: desired stance foot positions in Component Space

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float StepFrequencyHz = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float DutyFactor = 0.7f; // stance portion (0..1). Higher = more time planted.

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float StepTriggerDist = 18.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float StepHorizonSeconds = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float SwingHeight = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Ground")
	float TraceUp = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Ground")
	float TraceDown = 80.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Ground")
	TEnumAsByte<ECollisionChannel> GroundChannel = ECC_WorldStatic;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Ground")
	float FootClearance = 0.5f; // keep above ground along normal

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider|Runtime")
	TArray<FAG_LegRuntime> LegRuntime; // size 8

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider|Runtime")
	float Speed = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider|Runtime")
	FVector DesiredVelWS = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider|Runtime")
	float GaitPhase = 0.0f; // 0..1

private:
	UPROPERTY(Transient)
	USkeletalMeshComponent* SkelComp = nullptr;

	UPROPERTY(Transient)
	APawn* OwningPawn = nullptr;

	void EnsureSizes();
	void UpdatePhase(float DeltaSeconds);
	void UpdateVelocity(float DeltaSeconds);
	void UpdateLegs(float DeltaSeconds);

	bool TraceGroundAt(const FVector& QueryWS, FVector& OutPointWS, FVector& OutNormalWS) const;

	static float Wrap01(float X);
};