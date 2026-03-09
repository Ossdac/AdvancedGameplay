// ProceduralSpiderGaitComponent.cpp

#include "ProceduralSpiderGaitComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "GameFramework/Character.h"

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
		UpdateLeg(Leg, DeltaTime, 0.f);
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
	if (!Owner) return;

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (!Mesh) return;

	// Ensure bone/socket queries are valid right now
	Mesh->RefreshBoneTransforms();
	Mesh->FinalizeBoneTransform();

	const FTransform MeshWorld = Mesh->GetComponentTransform();

	for (FSpiderLegRuntime& Leg : Legs)
	{
		Leg.bInStance = true;
		Leg.bStepping = false;
		Leg.StepAlpha = 0.0f;

		const bool bHasFootSocket = !Leg.FootSocketName.IsNone() && Mesh->DoesSocketExist(Leg.FootSocketName);
		const FName FootName = bHasFootSocket ? Leg.FootSocketName : Leg.EndBoneName;

		// Current pose locations (world)
		const FVector FootWorld = Mesh->GetSocketLocation(FootName);
		const FVector TipWorld  = Mesh->GetSocketLocation(Leg.EndBoneName);

		// Convert both to mesh component space
		const FVector FootCS = MeshWorld.InverseTransformPosition(FootWorld);
		const FVector TipCS  = MeshWorld.InverseTransformPosition(TipWorld);

		// Cache offsets
		Leg.RestOffset_Component = FootCS;            // rest FOOT (socket) position in component space
		Leg.FootToTip_Component  = TipCS - FootCS;    // vector from foot socket to tarsus tip, in component space

		// Start planted at current pose (foot contact)
		Leg.PlantedWorld = FootWorld;

		// IMPORTANT: IK target must be the TIP target (tarsus), not the foot target
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

	// Use row 3 as the default gait basis since it is your main stride row.
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
	UWorld* World = GetWorld();
	if (!World) return false;

	const FVector Start = WorldFrom;
	const FVector End = WorldFrom - FVector(0.0f, 0.0f, TraceDown + TraceUp);

	FHitResult Hit;
	FCollisionQueryParams Params(SCENE_QUERY_STAT(SpiderFootTrace), false, GetOwner());

	const bool bHit = World->LineTraceSingleByChannel(Hit, Start, End, GroundTraceChannel, Params);
	if (!bHit) return false;

	OutHitPoint = Hit.ImpactPoint;
	OutHitNormal = Hit.ImpactNormal;
	return true;
}

FVector UProceduralSpiderGaitComponent::ComputeDesiredFootPoint(const FSpiderLegRuntime& Leg) const
{
	AActor* Owner = GetOwner();
	if (!Owner) return Leg.PlantedWorld;

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh = Character ? Character->GetMesh() : Owner->FindComponentByClass<USkeletalMeshComponent>();
	if (!Mesh) return Leg.PlantedWorld;

	const FTransform MeshWorld = Mesh->GetComponentTransform();

	// Reconstruct rest foot position from the stored component-space position
	FVector GuessWorld = MeshWorld.TransformPosition(Leg.RestOffset_Component);

	const FVector Vel = Owner->GetVelocity();
	FVector MoveDir = FVector(Vel.X, Vel.Y, 0.0f).GetSafeNormal();

	if (!MoveDir.IsNearlyZero())
	{
		const float StepLengthForThisLeg = GetStepLengthForLeg(Leg.LegId);
		GuessWorld += MoveDir * StepLengthForThisLeg;
	}

	FVector HitP, HitN;
	if (SampleGround(GuessWorld + FVector(0.0f, 0.0f, TraceUp), HitP, HitN))
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

void UProceduralSpiderGaitComponent::TickStep(
	FSpiderLegRuntime& Leg,
	float DeltaTime,
	float SwingDuration)
{
	Leg.StepAlpha += DeltaTime / FMath::Max(0.001f, SwingDuration);
	Leg.StepAlpha = FMath::Clamp(Leg.StepAlpha, 0.f, 1.f);

	const float A = Leg.StepAlpha;

	const FVector Flat =
		FMath::Lerp(Leg.StepStartWorld, Leg.StepEndWorld, A);

	const float StepHeightForThisLeg = GetStepHeightForLeg(Leg.LegId);

	const float Lift =
		FMath::Sin(A * PI) * StepHeightForThisLeg;

	const FVector FootPos =
		Flat + FVector(0, 0, Lift);

	AActor* Owner = GetOwner();
	if (!Owner) return;

	ACharacter* Character = Cast<ACharacter>(Owner);

	USkeletalMeshComponent* Mesh =
		Character ? Character->GetMesh() :
		Owner->FindComponentByClass<USkeletalMeshComponent>();

	if (!Mesh) return;

	const FTransform MeshWorld = Mesh->GetComponentTransform();

	const FVector TipPos =
		FootPos +
		MeshWorld.TransformVector(Leg.FootToTip_Component);

	Leg.IKTargetWorld = FTransform(FQuat::Identity, TipPos);

	if (Leg.StepAlpha >= 1.f)
	{
		Leg.bStepping = false;
		Leg.bInStance = true;
		Leg.bDoneThisGroup = true;

		Leg.PlantedWorld = Leg.StepEndWorld;

		const FVector TipPlanted =
			Leg.PlantedWorld +
			MeshWorld.TransformVector(Leg.FootToTip_Component);

		Leg.IKTargetWorld = FTransform(FQuat::Identity, TipPlanted);

		RemainingInGroup = FMath::Max(0, RemainingInGroup - 1);
	}
}

void UProceduralSpiderGaitComponent::UpdateLeg(
	FSpiderLegRuntime& Leg,
	float DeltaTime,
	float)
{
	AActor* Owner = GetOwner();
	if (!Owner) return;

	ACharacter* Character = Cast<ACharacter>(Owner);
	USkeletalMeshComponent* Mesh =
		Character ? Character->GetMesh() :
		Owner->FindComponentByClass<USkeletalMeshComponent>();

	if (!Mesh) return;

	const FTransform MeshWorld = Mesh->GetComponentTransform();

	// Desired foot contact point in world
	Leg.DesiredWorld = ComputeDesiredFootPoint(Leg);

	// If currently stepping, continue the step
	if (Leg.bStepping)
	{
		const float SwingDuration =
			CurrentCycleSeconds *
			FMath::Clamp(1.f - DutyFactor, 0.05f, 0.45f);

		TickStep(Leg, DeltaTime, SwingDuration);
		return;
	}

	// Otherwise keep the foot planted, but drive the tarsus tip target
	const FVector TipTarget =
		Leg.PlantedWorld +
		MeshWorld.TransformVector(Leg.FootToTip_Component);

	Leg.IKTargetWorld = FTransform(FQuat::Identity, TipTarget);

	if (bDrawFootTargets)
	{
		DrawDebugSphere(GetWorld(), Leg.PlantedWorld, 40.0f, 12, FColor::Red, false, 0.0f);
		DrawDebugSphere(GetWorld(), Leg.DesiredWorld, 40.0f, 12, FColor::Green, false, 0.0f);
		DrawDebugSphere(GetWorld(), TipTarget, 40.0f, 12, FColor::Cyan, false, 0.0f);
	}

	// No commanded motion -> do not start a new step
	if (CommandedSpeed < 1.0f)
	{
		return;
	}

	// Wrong group -> cannot start
	const bool bCorrectGroup = (IsLegInGroupA(Leg.LegId) == bGroupASwings);
	if (!bCorrectGroup)
	{
		return;
	}

	// Already finished this group -> wait for group swap
	if (Leg.bDoneThisGroup)
	{
		return;
	}

	// Start a step if far enough from desired position
	const float Dist = FVector::Dist2D(Leg.PlantedWorld, Leg.DesiredWorld);
	if (Dist > StepTriggerDistance)
	{
		BeginStep(Leg, Leg.DesiredWorld);
	}
}

bool UProceduralSpiderGaitComponent::IsLegInGroupA(ESpiderLeg LegId) const
{
	// GroupA: L1 L3 R2 R4
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