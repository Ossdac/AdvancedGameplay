#pragma once

#include "CoreMinimal.h"
#include "Animation/AnimInstance.h"
#include "AG_SpiderAnimInstance.generated.h"

USTRUCT(BlueprintType)
struct FSpiderLegPose
{
	GENERATED_BODY()

	// Rotation to apply to the leg root/hip bone (local space)
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FRotator HipLocalRotation = FRotator::ZeroRotator;
};

UCLASS()
class YOURGAME_API UAG_SpiderAnimInstance : public UAnimInstance
{
	GENERATED_BODY()

public:
	// 8 entries: L1..L4, R1..R4 (match your ordering)
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Spider")
	TArray<FSpiderLegPose> Legs;
};