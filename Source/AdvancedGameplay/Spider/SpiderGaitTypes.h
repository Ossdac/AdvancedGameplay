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

USTRUCT(BlueprintType)
struct FSpiderLegRuntime
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	ESpiderLeg LegId = ESpiderLeg::L1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName IKBoneName;          // e.g. Leg1_IKtarsus_L

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName EndBoneName;         // e.g. Leg1_tarsus_L

	UPROPERTY(BlueprintReadOnly)
	bool bInStance = true;

	UPROPERTY(BlueprintReadOnly)
	bool bStepping = false;

	UPROPERTY(BlueprintReadOnly)
	float PhaseOffset = 0.0f;  // per-leg offset within cycle

	UPROPERTY(BlueprintReadOnly)
	FVector PlantedWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FVector DesiredWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FTransform IKTargetWorld = FTransform::Identity;

	// Swing interpolation
	UPROPERTY(BlueprintReadOnly)
	float StepAlpha = 0.0f;

	UPROPERTY(BlueprintReadOnly)
	FVector StepStartWorld = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly)
	FVector StepEndWorld = FVector::ZeroVector;
	
	UPROPERTY(BlueprintReadOnly)
	FVector RestOffset_Component = FVector::ZeroVector;
	
};