#include "ProceduralSpiderGaitComponent.h"

#include "SpiderCharacter.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"
#include "Components/SkeletalMeshComponent.h"

UProceduralSpiderGaitComponent::UProceduralSpiderGaitComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UProceduralSpiderGaitComponent::BeginPlay()
{
	Super::BeginPlay();

	if (UWorld* World = GetWorld())
	{
		World->GetTimerManager().SetTimerForNextTick(this, &UProceduralSpiderGaitComponent::InitializeFromCurrentPose);
	}
}

void UProceduralSpiderGaitComponent::TickComponent(
	float DeltaTime,
	ELevelTick TickType,
	FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bInitializedFromPose)
	{
		return;
	}

	UpdateCycle(DeltaTime);

	for (FSpiderLegRuntime& Leg : Legs)
	{
		UpdateLeg(Leg, DeltaTime, 0.0f);
	}
}

void UProceduralSpiderGaitComponent::InitializeFromCurrentPose()
{
	if (bInitializedFromPose)
	{
		return;
	}

	InitializeDefaultsIfEmpty();

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (!Mesh)
	{
		return;
	}

	Mesh->RefreshBoneTransforms();
	Mesh->FinalizeBoneTransform();

	const FTransform MeshWorld = Mesh->GetComponentTransform();

	for (FSpiderLegRuntime& Leg : Legs)
	{
		Leg.bInStance = true;
		Leg.bStepping = false;
		Leg.bDoneThisGroup = false;
		Leg.StepAlpha = 0.0f;

		const bool bHasFootSocket = !Leg.FootSocketName.IsNone() && Mesh->DoesSocketExist(Leg.FootSocketName);
		const FName FootName = bHasFootSocket ? Leg.FootSocketName : Leg.EndBoneName;

		const FVector FootWorld = Mesh->GetSocketLocation(FootName);
		const FVector TipWorld = Mesh->GetSocketLocation(Leg.EndBoneName);

		const FVector FootCS = MeshWorld.InverseTransformPosition(FootWorld);
		const FVector TipCS = MeshWorld.InverseTransformPosition(TipWorld);

		Leg.RestOffset_Component = FootCS;
		Leg.FootToTip_Component = TipCS - FootCS;

		Leg.PlantedWorld = FootWorld;

		const FVector TipTargetWorld = MeshWorld.TransformPosition(FootCS + Leg.FootToTip_Component);
		Leg.IKTargetWorld = FTransform(FQuat::Identity, TipTargetWorld);
	}

	bInitializedFromPose = true;
}

void UProceduralSpiderGaitComponent::InitializeDefaultsIfEmpty()
{
	if (Legs.Num() != 8)
	{
		Legs.Empty();
		Legs.SetNum(8);

		Legs[0].LegId = ESpiderLeg::L1; Legs[0].IKBoneName = TEXT("Leg1_IKtarsus_L"); Legs[0].EndBoneName = TEXT("Leg1_tarsus_L"); Legs[0].FootSocketName = TEXT("L1_FootSocket");
		Legs[1].LegId = ESpiderLeg::L2; Legs[1].IKBoneName = TEXT("Leg2_IKtarsus_L"); Legs[1].EndBoneName = TEXT("Leg2_tarsus_L"); Legs[1].FootSocketName = TEXT("L2_FootSocket");
		Legs[2].LegId = ESpiderLeg::L3; Legs[2].IKBoneName = TEXT("Leg3_IKtarsus_L"); Legs[2].EndBoneName = TEXT("Leg3_tarsus_L"); Legs[2].FootSocketName = TEXT("L3_FootSocket");
		Legs[3].LegId = ESpiderLeg::L4; Legs[3].IKBoneName = TEXT("Leg4_IKtarsus_L"); Legs[3].EndBoneName = TEXT("Leg4_tarsus_L"); Legs[3].FootSocketName = TEXT("L4_FootSocket");

		Legs[4].LegId = ESpiderLeg::R1; Legs[4].IKBoneName = TEXT("Leg1_IKtarsus_R"); Legs[4].EndBoneName = TEXT("Leg1_tarsus_R"); Legs[4].FootSocketName = TEXT("R1_FootSocket");
		Legs[5].LegId = ESpiderLeg::R2; Legs[5].IKBoneName = TEXT("Leg2_IKtarsus_R"); Legs[5].EndBoneName = TEXT("Leg2_tarsus_R"); Legs[5].FootSocketName = TEXT("R2_FootSocket");
		Legs[6].LegId = ESpiderLeg::R3; Legs[6].IKBoneName = TEXT("Leg3_IKtarsus_R"); Legs[6].EndBoneName = TEXT("Leg3_tarsus_R"); Legs[6].FootSocketName = TEXT("R3_FootSocket");
		Legs[7].LegId = ESpiderLeg::R4; Legs[7].IKBoneName = TEXT("Leg4_IKtarsus_R"); Legs[7].EndBoneName = TEXT("Leg4_tarsus_R"); Legs[7].FootSocketName = TEXT("R4_FootSocket");
	}
}

void UProceduralSpiderGaitComponent::UpdateCycle(float DeltaTime)
{
	const float Speed = FMath::Max(CommandedSpeed, 0.0f);
	if (Speed < 1.0f)
	{
		return;
	}

	const float GaitBasisStepLength = FMath::Max(1.0f, StepLength3);
	const float StepFrequency = Speed / GaitBasisStepLength;
	CurrentCycleSeconds = 1.0f / FMath::Max(StepFrequency, 0.01f);

	if (RemainingInGroup > 0)
	{
		return;
	}

	bGroupASwings = !bGroupASwings;
	RemainingInGroup = 0;

	for (FSpiderLegRuntime& Leg : Legs)
	{
		const bool bInActiveGroup = (IsLegInGroupA(Leg.LegId) == bGroupASwings);
		if (bInActiveGroup)
		{
			Leg.bDoneThisGroup = false;
			RemainingInGroup++;
		}
	}
}

bool UProceduralSpiderGaitComponent::SampleGround(const FVector& WorldFrom, FVector& OutHitPoint, FVector& OutHitNormal) const
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return false;
	}

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	UWorld* World = GetWorld();

	if (!Mesh || !World)
	{
		return false;
	}

	const FVector Up = Mesh->GetUpVector().GetSafeNormal();
	const FVector Start = WorldFrom + Up * TraceUp;
	const FVector End = WorldFrom - Up * TraceDown;

	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(SpiderFootTrace), false, Owner);

	const bool bHit = World->LineTraceSingleByChannel(Hit, Start, End, GroundTraceChannel, Params);
	if (!bHit)
	{
		return false;
	}

	OutHitPoint = Hit.ImpactPoint;
	OutHitNormal = Hit.ImpactNormal;
	return true;
}

FVector UProceduralSpiderGaitComponent::ComputeDesiredFootPoint(const FSpiderLegRuntime& Leg) const
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return Leg.PlantedWorld;
	}

	ASpiderCharacter* Spider = Cast<ASpiderCharacter>(Owner);
	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (!Mesh)
	{
		return Leg.PlantedWorld;
	}

	const FTransform MeshWorld = Mesh->GetComponentTransform();
	const FVector Up = MeshWorld.GetUnitAxis(EAxis::Z).GetSafeNormal();

	FVector GuessWorld = MeshWorld.TransformPosition(Leg.RestOffset_Component);

	FVector Vel = Spider ? Spider->GetMovementVelocity() : Owner->GetVelocity();
	FVector PlanarVel = FVector::VectorPlaneProject(Vel, Up);
	FVector MoveDir = PlanarVel.GetSafeNormal();

	const float StepLengthForLeg = GetStepLengthForLeg(Leg.LegId);
	if (!MoveDir.IsNearlyZero())
	{
		GuessWorld += MoveDir * StepLengthForLeg;
	}

	FVector HitP;
	FVector HitN;
	if (SampleGround(GuessWorld, HitP, HitN))
	{
		const float SnapLimit = (Leg.MaxSnapToGround > 0.0f) ? Leg.MaxSnapToGround : MaxSnapToGround;
		if (FVector::Dist(HitP, GuessWorld) <= SnapLimit)
		{
			return HitP;
		}
	}

	return GuessWorld;
}

void UProceduralSpiderGaitComponent::BeginStep(FSpiderLegRuntime& Leg, const FVector& NewEndWorld)
{
	Leg.bStepping = true;
	Leg.bInStance = false;
	Leg.StepAlpha = 0.0f;
	Leg.StepStartWorld = Leg.PlantedWorld;
	Leg.StepEndWorld = NewEndWorld;
}

void UProceduralSpiderGaitComponent::TickStep(FSpiderLegRuntime& Leg, float DeltaTime, float SwingDuration)
{
	Leg.StepAlpha += DeltaTime / FMath::Max(0.001f, SwingDuration);
	Leg.StepAlpha = FMath::Clamp(Leg.StepAlpha, 0.0f, 1.0f);

	const float A = Leg.StepAlpha;

	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (!Mesh)
	{
		return;
	}

	const FTransform MeshWorld = Mesh->GetComponentTransform();
	const FVector Up = MeshWorld.GetUnitAxis(EAxis::Z).GetSafeNormal();

	const FVector Flat = FMath::Lerp(Leg.StepStartWorld, Leg.StepEndWorld, A);
	const float StepHeightForThisLeg = GetStepHeightForLeg(Leg.LegId);
	const float Lift = FMath::Sin(A * PI) * StepHeightForThisLeg;

	const FVector FootPos = Flat + Up * Lift;
	const FVector TipPos = FootPos + MeshWorld.TransformVector(Leg.FootToTip_Component);

	Leg.IKTargetWorld = FTransform(FQuat::Identity, TipPos);

	if (Leg.StepAlpha >= 1.0f)
	{
		Leg.bStepping = false;
		Leg.bInStance = true;
		Leg.bDoneThisGroup = true;
		Leg.PlantedWorld = Leg.StepEndWorld;

		const FVector TipPlanted = Leg.PlantedWorld + MeshWorld.TransformVector(Leg.FootToTip_Component);
		Leg.IKTargetWorld = FTransform(FQuat::Identity, TipPlanted);

		RemainingInGroup = FMath::Max(0, RemainingInGroup - 1);
	}
}

void UProceduralSpiderGaitComponent::UpdateLeg(FSpiderLegRuntime& Leg, float DeltaTime, float)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (!Mesh)
	{
		return;
	}

	const FTransform MeshWorld = Mesh->GetComponentTransform();

	Leg.DesiredWorld = ComputeDesiredFootPoint(Leg);

	if (Leg.bStepping)
	{
		const float SwingDuration = CurrentCycleSeconds * FMath::Clamp(1.0f - DutyFactor, 0.05f, 0.45f);
		TickStep(Leg, DeltaTime, SwingDuration);
		return;
	}

	const FVector TipTarget = Leg.PlantedWorld + MeshWorld.TransformVector(Leg.FootToTip_Component);
	Leg.IKTargetWorld = FTransform(FQuat::Identity, TipTarget);

	if (bDrawFootTargets)
	{
		DrawDebugSphere(GetWorld(), Leg.PlantedWorld, 4.0f, 8, FColor::Red, false, 0.0f);
		DrawDebugSphere(GetWorld(), Leg.DesiredWorld, 4.0f, 8, FColor::Green, false, 0.0f);
		DrawDebugSphere(GetWorld(), TipTarget, 4.0f, 8, FColor::Cyan, false, 0.0f);
	}

	if (CommandedSpeed < 1.0f)
	{
		return;
	}

	const bool bCorrectGroup = (IsLegInGroupA(Leg.LegId) == bGroupASwings);
	if (!bCorrectGroup)
	{
		return;
	}

	if (Leg.bDoneThisGroup)
	{
		return;
	}

	const FVector Up = MeshWorld.GetUnitAxis(EAxis::Z).GetSafeNormal();
	const FVector DeltaToDesired = FVector::VectorPlaneProject(Leg.DesiredWorld - Leg.PlantedWorld, Up);
	const float Dist = DeltaToDesired.Size();

	if (Dist > StepTriggerDistance)
	{
		BeginStep(Leg, Leg.DesiredWorld);
	}
}

bool UProceduralSpiderGaitComponent::IsLegInGroupA(ESpiderLeg LegId) const
{
	return (LegId == ESpiderLeg::L1) || (LegId == ESpiderLeg::L3) || (LegId == ESpiderLeg::R2) || (LegId == ESpiderLeg::R4);
}

bool UProceduralSpiderGaitComponent::GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const
{
	if (!bInitializedFromPose)
	{
		return false;
	}

	for (const FSpiderLegRuntime& L : Legs)
	{
		if (L.LegId == Leg)
		{
			OutWorldTarget = L.IKTargetWorld;
			return true;
		}
	}

	return false;
}

float UProceduralSpiderGaitComponent::GetStepLengthForLeg(ESpiderLeg LegId) const
{
	switch (LegId)
	{
	case ESpiderLeg::L1:
	case ESpiderLeg::R1:
		return StepLength1;

	case ESpiderLeg::L2:
	case ESpiderLeg::R2:
		return StepLength2;

	case ESpiderLeg::L3:
	case ESpiderLeg::R3:
		return StepLength3;

	case ESpiderLeg::L4:
	case ESpiderLeg::R4:
		return StepLength4;

	default:
		return StepLength3;
	}
}

float UProceduralSpiderGaitComponent::GetStepHeightForLeg(ESpiderLeg LegId) const
{
	switch (LegId)
	{
	case ESpiderLeg::L1:
	case ESpiderLeg::R1:
		return StepHeight1;

	case ESpiderLeg::L2:
	case ESpiderLeg::R2:
		return StepHeight2;

	case ESpiderLeg::L3:
	case ESpiderLeg::R3:
		return StepHeight3;

	case ESpiderLeg::L4:
	case ESpiderLeg::R4:
		return StepHeight4;

	default:
		return StepHeight3;
	}
}