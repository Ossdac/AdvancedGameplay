#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "SpiderGaitTypes.h"
#include "SpiderAnimInstance.generated.h"

class UProceduralSpiderGaitComponent;

UCLASS()
class ADVANCEDGAMEPLAY_API USpiderAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

public:
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;

	// Optional: expose a getter instead of a TMap for easier AnimGraph wiring later.
	UFUNCTION(BlueprintCallable, BlueprintPure)
	bool GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const;

protected:
	UPROPERTY(Transient, BlueprintReadOnly)
	TObjectPtr<UProceduralSpiderGaitComponent> SpiderGait;

	// A TMap is awkward to use in AnimGraph/Blueprint. Works in C++, but pain in BP.
	UPROPERTY(Transient, BlueprintReadOnly)
	TArray<FTransform> IKTargets; // size 8, index = (int)ESpiderLeg
};