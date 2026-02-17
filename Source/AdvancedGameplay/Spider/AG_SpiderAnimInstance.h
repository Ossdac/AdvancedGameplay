#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "AG_SpiderAnimInstance.generated.h"

UCLASS()
class ADVANCEDGAMEPLAY_API UAG_SpiderAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

public:
	virtual void NativeInitializeAnimation() override;
	virtual void NativeUpdateAnimation(float DeltaSeconds) override;

	// 8 legs: 0..3 left (Leg1..Leg4), 4..7 right (Leg1..Leg4)
	UPROPERTY(BlueprintReadOnly, Category="Spider|Gait")
	TArray<float> LegSwingDegrees;

	UPROPERTY(BlueprintReadOnly, Category="Spider|Gait")
	bool bIsMoving = false;

	UPROPERTY(BlueprintReadOnly, Category="Spider|Gait")
	float Speed = 0.0f;

protected:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float MaxSwingDeg = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float StepFrequencyHz = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider|Gait")
	float SpeedForFullGait = 250.0f;

private:
	UPROPERTY(Transient)
	APawn* OwningPawn = nullptr;

	float GaitTime = 0.0f;
};