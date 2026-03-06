#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "SpiderCharacter.generated.h"

class UProceduralSpiderGaitComponent;
class USpringArmComponent;
class UCameraComponent;

UCLASS()
class ADVANCEDGAMEPLAY_API ASpiderCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	ASpiderCharacter();

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	void HandleRawMovement();

public:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Spider")
	TObjectPtr<UProceduralSpiderGaitComponent> SpiderGait;

	UPROPERTY(BlueprintReadOnly, Category="Camera")
	TObjectPtr<USpringArmComponent> SpringArm = nullptr;

	UPROPERTY(BlueprintReadOnly, Category="Camera")
	TObjectPtr<UCameraComponent> Camera = nullptr;
};