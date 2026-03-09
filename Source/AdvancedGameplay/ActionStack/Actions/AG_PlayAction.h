#pragma once

#include "CoreMinimal.h"
#include "AG_ActionBase.h"
#include "AG_PlayAction.generated.h"

class ASpiderCharacter;
class AActor;

UCLASS()
class ADVANCEDGAMEPLAY_API UAG_PlayAction : public UAG_ActionBase
{
	GENERATED_BODY()

public:
	virtual void OnBegin(bool bFirstTime) override;
	virtual void OnUpdate() override;
	virtual void OnEnd() override;
	virtual bool IsDone() const override;

private:
	bool bDone = false;
	bool bResultShown = false;

	// Change these numbers in code if needed.
	float BallWinY = -1200.0f;
	float SpiderLoseY = -1200.0f;

	TWeakObjectPtr<AActor> BallActor;
	TWeakObjectPtr<ASpiderCharacter> SpiderActor;

	void FindActors();
	void ShowResult(const FString& ResultText);
};