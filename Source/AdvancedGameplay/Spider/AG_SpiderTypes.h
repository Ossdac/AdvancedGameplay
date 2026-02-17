#pragma once

#include "CoreMinimal.h"
#include "AG_SpiderTypes.generated.h"

UENUM(BlueprintType)
enum class EAG_LegPhase : uint8
{
	Stance UMETA(DisplayName="Stance"),
	Swing  UMETA(DisplayName="Swing")
};

USTRUCT(BlueprintType)
struct FAG_JointLimit
{
	GENERATED_BODY()

	// Bone-local axis used as twist axis (unit vector in bone local space).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Limits")
	FVector TwistAxisLS = FVector(1.0f, 0.0f, 0.0f);

	// Twist clamp (degrees) around TwistAxisLS
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Limits")
	float TwistMinDeg = -10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Limits")
	float TwistMaxDeg =  10.0f;

	// Maximum swing angle away from TwistAxisLS (degrees).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Limits")
	float SwingMaxDeg = 25.0f;
};

USTRUCT(BlueprintType)
struct FAG_LegBones
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Coxa;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Trochanter;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Femur;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Patella;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Tibia;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Metatarsus;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Bones") FName Tarsus;
};

USTRUCT(BlueprintType)
struct FAG_LegRuntime
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	EAG_LegPhase Phase = EAG_LegPhase::Stance;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	float PhaseT = 0.0f; // 0..1

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Runtime")
	float PhaseOffset = 0.0f; // 0..1

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	bool bFootHasLock = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	FVector FootLockWS = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	FVector FootTargetWS = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	FVector FootCurrentWS = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	bool bHasGround = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	FVector GroundPointWS = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Runtime")
	FVector GroundNormalWS = FVector::UpVector;
};