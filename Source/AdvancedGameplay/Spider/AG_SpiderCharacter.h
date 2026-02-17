#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "AG_SpiderCharacter.generated.h"

UCLASS()
class ADVANCEDGAMEPLAY_API AAG_SpiderCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	AAG_SpiderCharacter();

protected:
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

private:
	void MoveForward(float Value);
	void MoveRight(float Value);
	void Turn(float Value);
	void LookUp(float Value);

	UPROPERTY(EditAnywhere, Category="Spider|Movement")
	float TurnRateDegPerSec = 180.0f;

	UPROPERTY(EditAnywhere, Category="Spider|Movement")
	float LookUpRateDegPerSec = 90.0f;
};