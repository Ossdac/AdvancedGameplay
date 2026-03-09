#pragma once

#include "CoreMinimal.h"
#include "AG_ActionBase.h"
#include "AG_ResultAction.generated.h"

UCLASS()
class ADVANCEDGAMEPLAY_API UAG_ResultAction : public UAG_ActionBase
{
	GENERATED_BODY()

public:
	UPROPERTY()
	FString ResultText = TEXT("CONGRATULATIONS");

	virtual void OnBegin(bool bFirstTime) override;
	virtual void OnUpdate() override;
	virtual void OnEnd() override;
	virtual bool IsDone() const override;

private:
	bool bDone = false;
};