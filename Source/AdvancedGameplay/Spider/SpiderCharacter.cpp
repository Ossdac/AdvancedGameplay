#include "SpiderCharacter.h"
#include "ProceduralSpiderGaitComponent.h"

#include "Camera/CameraComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "Components/InputComponent.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/PlayerController.h"
#include "InputCoreTypes.h"
#include "AdvancedGameplay/Physics/AG_RigidbodyComponent.h"

ASpiderCharacter::ASpiderCharacter()
{
	PrimaryActorTick.bCanEverTick = true;

	bUseControllerRotationYaw = false;
	bUseControllerRotationPitch = false;
	bUseControllerRotationRoll = false;

	GetCharacterMovement()->bOrientRotationToMovement = true;
	GetCharacterMovement()->RotationRate = FRotator(0.0f, RotationRateDegrees, 0.0f);
	GetCharacterMovement()->MaxWalkSpeed = MaxWalkSpeed;

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

	UCapsuleComponent* Capsule = GetCapsuleComponent();
	if (Capsule)
	{
		Capsule->SetNotifyRigidBodyCollision(true);
		Capsule->OnComponentHit.AddDynamic(this, &ASpiderCharacter::OnSpiderHit);
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

	if (PlayerController->IsInputKeyDown(EKeys::W))
	{
		ForwardValue += 1.0f;
	}

	if (!FMath::IsNearlyZero(ForwardValue))
	{
		const FRotator ControlRotation = Controller->GetControlRotation();
		const FRotator YawRotation(0.0f, ControlRotation.Yaw, 0.0f);
		const FVector ForwardDirection = FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X);
		AddMovementInput(ForwardDirection, ForwardValue);
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

void ASpiderCharacter::OnSpiderHit(
	UPrimitiveComponent* HitComponent,
	AActor* OtherActor,
	UPrimitiveComponent* OtherComp,
	FVector NormalImpulse,
	const FHitResult& Hit)
{
	if (!OtherActor || OtherActor == this)
	{
		return;
	}

	UAG_RigidbodyComponent* OtherBody = OtherActor->FindComponentByClass<UAG_RigidbodyComponent>();
	if (!OtherBody)
	{
		return;
	}

	FVector PushDirection = GetVelocity().GetSafeNormal();

	if (PushDirection.IsNearlyZero())
	{
		PushDirection = GetActorForwardVector();
	}

	OtherBody->AddForce(PushDirection * PushForce);
}