#include "SpiderCharacter.h"
#include "ProceduralSpiderGaitComponent.h"

#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"

ASpiderCharacter::ASpiderCharacter()
{
	PrimaryActorTick.bCanEverTick = true;

	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;

	GetCharacterMovement()->bOrientRotationToMovement = true;
	GetCharacterMovement()->RotationRate = FRotator(0.0f, 360.0f, 0.0f);
	GetCharacterMovement()->MaxWalkSpeed = 400.0f;

	SpiderGait = CreateDefaultSubobject<UProceduralSpiderGaitComponent>(TEXT("SpiderGait"));
}

void ASpiderCharacter::BeginPlay()
{
	Super::BeginPlay();

	SpringArm = FindComponentByClass<USpringArmComponent>();
	Camera = FindComponentByClass<UCameraComponent>();

	APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if (PlayerController)
	{
		PlayerController->SetInputMode(FInputModeGameOnly());
		PlayerController->bShowMouseCursor = false;
	}
}

void ASpiderCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	HandleRawMovement();

	if (SpiderGait)
	{
		const FVector Velocity = GetVelocity();
		const float Speed2D = FVector(Velocity.X, Velocity.Y, 0.0f).Size();
		SpiderGait->CommandedSpeed = Speed2D;
	}
}

void ASpiderCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}

void ASpiderCharacter::HandleRawMovement()
{
	APlayerController* PlayerController = Cast<APlayerController>(GetController());
	if (!PlayerController || !Controller)
	{
		return;
	}

	float ForwardValue = 0.0f;
	float RightValue = 0.0f;

	if (PlayerController->IsInputKeyDown(EKeys::W))
	{
		ForwardValue += 1.0f;
	}
	if (PlayerController->IsInputKeyDown(EKeys::S))
	{
		ForwardValue -= 1.0f;
	}
	if (PlayerController->IsInputKeyDown(EKeys::D))
	{
		RightValue += 1.0f;
	}
	if (PlayerController->IsInputKeyDown(EKeys::A))
	{
		RightValue -= 1.0f;
	}

	if (!FMath::IsNearlyZero(ForwardValue))
	{
		const FRotator ControlRotation = Controller->GetControlRotation();
		const FRotator YawRotation(0.0f, ControlRotation.Yaw, 0.0f);
		const FVector ForwardDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
		AddMovementInput(ForwardDirection, ForwardValue);
	}

	if (!FMath::IsNearlyZero(RightValue))
	{
		const FRotator ControlRotation = Controller->GetControlRotation();
		const FRotator YawRotation(0.0f, ControlRotation.Yaw, 0.0f);
		const FVector RightDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y);
		AddMovementInput(RightDirection, RightValue);
	}

	float MouseX = 0.0f;
	float MouseY = 0.0f;
	PlayerController->GetInputMouseDelta(MouseX, MouseY);

	if (!FMath::IsNearlyZero(MouseX))
	{
		AddControllerYawInput(MouseX);
	}

	if (!FMath::IsNearlyZero(MouseY))
	{
		AddControllerPitchInput(-MouseY);
	}
}