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
	
	virtual void NativeInitializeAnimation() override;

	// Optional: expose a getter instead of a TMap for easier AnimGraph wiring later.
	UFUNCTION(BlueprintCallable, BlueprintPure)
	bool GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const;
	
	UFUNCTION(BlueprintCallable, BlueprintPure)
	bool HasValidTargets() const { return bTargetsValid; }
	
	UPROPERTY(Transient, BlueprintReadOnly, Category="Spider|IK")
	bool bTargetsValid = false;// size 8, index = (int)ESpiderLeg

protected:
	UPROPERTY(Transient, BlueprintReadOnly)
	TObjectPtr<UProceduralSpiderGaitComponent> SpiderGait;

	// A TMap is awkward to use in AnimGraph/Blueprint. Works in C++, but pain in BP.
	UPROPERTY(Transient, BlueprintReadOnly)
	TArray<FTransform> IKTargets;
	
	
	
private:
	
};