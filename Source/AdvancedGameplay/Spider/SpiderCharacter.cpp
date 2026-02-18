#include "SpiderCharacter.h"
#include "ProceduralSpiderGaitComponent.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/CharacterMovementComponent.h"

ASpiderCharacter::ASpiderCharacter()
{
	PrimaryActorTick.bCanEverTick = true;

	// Character defaults that usually help for creatures
	GetCharacterMovement()->bOrientRotationToMovement = true;
	GetCharacterMovement()->RotationRate = FRotator(0.0f, 540.0f, 0.0f);

	// You can tune these later
	GetCharacterMovement()->MaxWalkSpeed = 200.0f;
	GetCharacterMovement()->BrakingDecelerationWalking = 2048.0f;

	SpiderGait = CreateDefaultSubobject<UProceduralSpiderGaitComponent>(TEXT("ProceduralSpiderGait"));
}

void ASpiderCharacter::BeginPlay()
{
	Super::BeginPlay();
}