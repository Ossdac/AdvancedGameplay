// SpiderGaitTypes.h
#pragma once

#include "CoreMinimal.h"
#include "SpiderGaitTypes.generated.h"

UENUM(BlueprintType)
enum class ESpiderLeg : uint8
{
	L1, L2, L3, L4,
	R1, R2, R3, R4
};

// SpiderGaitTypes.h

USTRUCT(BlueprintType)
struct FSpiderLegRuntime
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESpiderLeg LegId = ESpiderLeg::L1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName IKBoneName;

	// This is still the IK chain tip (bone), used by FABRIK "Tip Bone"
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName EndBoneName;

	// NEW: this is the actual foot contact point you create in Skeleton as a Socket
	// e.g. "L1_FootSocket"
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName FootSocketName;

	// (Optional) allow per-leg snap distance
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	float MaxSnapToGround = 60.0f;

	UPROPERTY(BlueprintReadOnly)
	bool bInStance = true;

	UPROPERTY(BlueprintReadOnly)
	bool bStepping = false;

	UPROPERTY(BlueprintReadOnly)
	float PhaseOffset = 0.0f;

	UPROPERTY(BlueprintReadOnly)
	FVector PlantedWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FVector DesiredWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FTransform IKTargetWorld = FTransform::Identity;

	UPROPERTY(BlueprintReadOnly)
	float StepAlpha = 0.0f;

	UPROPERTY(BlueprintReadOnly)
	FVector StepStartWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FVector StepEndWorld = FVector::ZeroVector;

	// Stored in MESH COMPONENT SPACE (not actor space)
	UPROPERTY(BlueprintReadOnly)
	FVector RestOffset_Component = FVector::ZeroVector;
	
	UPROPERTY(BlueprintReadOnly)
	FVector FootToTip_Component = FVector::ZeroVector;
	
	UPROPERTY(BlueprintReadOnly)
	bool bDoneThisGroup = false;
};