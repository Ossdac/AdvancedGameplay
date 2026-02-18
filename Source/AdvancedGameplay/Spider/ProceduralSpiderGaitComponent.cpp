#include "ProceduralSpiderGaitComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

UProceduralSpiderGaitComponent::UProceduralSpiderGaitComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
}

void UProceduralSpiderGaitComponent::BeginPlay()
{
	Super::BeginPlay();
	InitializeDefaultsIfEmpty();
}

void UProceduralSpiderGaitComponent::InitializeDefaultsIfEmpty()
{
	if (Legs.Num() != 8)
	{
		Legs.Empty();
		Legs.SetNum(8);

		Legs[0].LegId = ESpiderLeg::L1; Legs[0].IKBoneName = TEXT("Leg1_IKtarsus_L"); Legs[0].EndBoneName = TEXT("Leg1_tarsus_L");
		Legs[1].LegId = ESpiderLeg::L2; Legs[1].IKBoneName = TEXT("Leg2_IKtarsus_L"); Legs[1].EndBoneName = TEXT("Leg2_tarsus_L");
		Legs[2].LegId = ESpiderLeg::L3; Legs[2].IKBoneName = TEXT("Leg3_IKtarsus_L"); Legs[2].EndBoneName = TEXT("Leg3_tarsus_L");
		Legs[3].LegId = ESpiderLeg::L4; Legs[3].IKBoneName = TEXT("Leg4_IKtarsus_L"); Legs[3].EndBoneName = TEXT("Leg4_tarsus_L");

		Legs[4].LegId = ESpiderLeg::R1; Legs[4].IKBoneName = TEXT("Leg1_IKtarsus_R"); Legs[4].EndBoneName = TEXT("Leg1_tarsus_R");
		Legs[5].LegId = ESpiderLeg::R2; Legs[5].IKBoneName = TEXT("Leg2_IKtarsus_R"); Legs[5].EndBoneName = TEXT("Leg2_tarsus_R");
		Legs[6].LegId = ESpiderLeg::R3; Legs[6].IKBoneName = TEXT("Leg3_IKtarsus_R"); Legs[6].EndBoneName = TEXT("Leg3_tarsus_R");
		Legs[7].LegId = ESpiderLeg::R4; Legs[7].IKBoneName = TEXT("Leg4_IKtarsus_R"); Legs[7].EndBoneName = TEXT("Leg4_tarsus_R");
	}

	// Start planted under the body projection
	AActor* Owner = GetOwner();
	const FVector Base = Owner ? Owner->GetActorLocation() : FVector::ZeroVector;

	for (FSpiderLegRuntime& Leg : Legs)
	{
		Leg.bInStance = true;
		Leg.bStepping = false;
		Leg.StepAlpha = 0.0f;

		FVector HitP, HitN;
		const FVector From = Base + FVector(0,0,TraceUp);
		if (SampleGround(From, HitP, HitN))
		{
			Leg.PlantedWorld = HitP;
			Leg.IKTargetWorld = FTransform(Owner->GetActorRotation(), HitP);
		}
	}
}

void UProceduralSpiderGaitComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	UpdateCycle(DeltaTime);

	const float Cycle01 = FMath::Fmod(AccumTime / FMath::Max(0.001f, CycleSeconds), 1.0f);

	for (FSpiderLegRuntime& Leg : Legs)
	{
		UpdateLeg(Leg, DeltaTime, Cycle01);
	}
}

void UProceduralSpiderGaitComponent::UpdateCycle(float DeltaTime)
{
	AccumTime += DeltaTime;

	const float Cycle01 = FMath::Fmod(AccumTime / FMath::Max(0.001f, CycleSeconds), 1.0f);

	// Toggle swing group at half cycle boundaries
	const int32 Half = (Cycle01 < 0.5f) ? 0 : 1;
	if (Half != PrevHalf)
	{
		PrevHalf = Half;
		bGroupASwings = (Half == 0);
	}
}

bool UProceduralSpiderGaitComponent::IsLegInGroupA(ESpiderLeg LegId) const
{
	// Alternating tetrapod example group.
	// GroupA: L1 L3 R2 R4 (diagonal-ish pairing)
	return (LegId == ESpiderLeg::L1) || (LegId == ESpiderLeg::L3) || (LegId == ESpiderLeg::R2) || (LegId == ESpiderLeg::R4);
}

bool UProceduralSpiderGaitComponent::IsLegAllowedToSwing(const FSpiderLegRuntime& Leg, float Cycle01) const
{
	// Swing window within the current half-cycle.
	// Stance fraction = DutyFactor; swing fraction = 1-DutyFactor (duty > 0.6).	
	const float SwingFrac = FMath::Clamp(1.0f - DutyFactor, 0.05f, 0.45f);

	// In each half (0..0.5 or 0.5..1), allow swing near the beginning of that half.
	const float HalfStart = bGroupASwings ? 0.0f : 0.5f;
	const float Local = (Cycle01 - HalfStart) * 2.0f; // map to 0..1 inside half
	if (Local < 0.0f || Local > 1.0f) return false;

	const bool bInThisGroup = bGroupASwings ? IsLegInGroupA(Leg.LegId) : !IsLegInGroupA(Leg.LegId);

	return bInThisGroup && (Local <= SwingFrac);
}

bool UProceduralSpiderGaitComponent::SampleGround(const FVector& WorldFrom, FVector& OutHitPoint, FVector& OutHitNormal) const
{
	UWorld* World = GetWorld();
	if (!World) return false;

	const FVector Start = WorldFrom;
	const FVector End   = WorldFrom - FVector(0,0,TraceDown + TraceUp);

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

	// Very simple desired placement: ahead of body in movement direction (replace with actual velocity).
	const FVector Forward = Owner->GetActorForwardVector();
	const FVector Right   = Owner->GetActorRightVector();

	// Spread feet laterally based on which side and leg index (rough heuristic).
	const bool bLeft = (Leg.LegId == ESpiderLeg::L1 || Leg.LegId == ESpiderLeg::L2 || Leg.LegId == ESpiderLeg::L3 || Leg.LegId == ESpiderLeg::L4);
	const float SideSign = bLeft ? 1.0f : -1.0f;

	float Along = 0.0f;
	switch (Leg.LegId)
	{
		case ESpiderLeg::L1: case ESpiderLeg::R1: Along =  18.0f; break;
		case ESpiderLeg::L2: case ESpiderLeg::R2: Along =   6.0f; break;
		case ESpiderLeg::L3: case ESpiderLeg::R3: Along =  -6.0f; break;
		case ESpiderLeg::L4: case ESpiderLeg::R4: Along = -18.0f; break;
	}

	const FVector Body = Owner->GetActorLocation();

	const FVector Velocity = Owner->GetVelocity();
	const float Speed2D = FVector(Velocity.X, Velocity.Y, 0.0f).Size();
	const float ForwardReach = (Speed2D > 2.0f) ? StepReachForward : 0.0f;

	const FVector Guess = Body + Forward * (Along + ForwardReach) + Right * (SideSign * 18.0f);

	FVector HitP, HitN;
	if (SampleGround(Guess + FVector(0,0,TraceUp), HitP, HitN))
	{
		return HitP;
	}
	return Leg.PlantedWorld;
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
	Leg.StepAlpha = FMath::Clamp(Leg.StepAlpha + (DeltaTime / FMath::Max(0.001f, SwingDuration)), 0.0f, 1.0f);

	// Arc lift
	const float A = Leg.StepAlpha;
	const FVector Flat = FMath::Lerp(Leg.StepStartWorld, Leg.StepEndWorld, A);
	const float Lift = FMath::Sin(A * PI) * StepHeight;
	const FVector Pos = Flat + FVector(0,0,Lift);

	Leg.IKTargetWorld = FTransform(GetOwner()->GetActorRotation(), Pos);

	if (Leg.StepAlpha >= 1.0f)
	{
		Leg.bStepping = false;
		Leg.bInStance = true;
		Leg.PlantedWorld = Leg.StepEndWorld;
		Leg.IKTargetWorld = FTransform(GetOwner()->GetActorRotation(), Leg.PlantedWorld);
	}
}

void UProceduralSpiderGaitComponent::UpdateLeg(FSpiderLegRuntime& Leg, float DeltaTime, float Cycle01)
{
	const float SwingDuration = CycleSeconds * FMath::Clamp(1.0f - DutyFactor, 0.05f, 0.45f);

	Leg.DesiredWorld = ComputeDesiredFootPoint(Leg);

	if (Leg.bStepping)
	{
		TickStep(Leg, DeltaTime, SwingDuration);
		return;
	}

	// stance: lock target to planted point
	Leg.IKTargetWorld = FTransform(GetOwner()->GetActorRotation(), Leg.PlantedWorld);

	// if allowed to swing this tick, and foot needs repositioning => step
	if (IsLegAllowedToSwing(Leg, Cycle01))
	{
		const float Dist = FVector::Dist2D(Leg.PlantedWorld, Leg.DesiredWorld);
		if (Dist > StepTriggerDistance)
		{
			BeginStep(Leg, Leg.DesiredWorld);
		}
	}
}

bool UProceduralSpiderGaitComponent::GetIKTarget(ESpiderLeg Leg, FTransform& OutWorldTarget) const
{
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