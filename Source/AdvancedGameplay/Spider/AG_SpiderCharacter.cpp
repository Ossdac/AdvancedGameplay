// Fill out your copyright notice in the Description page of Project Settings.


#include "Spider/AG_SpiderCharacter.h"

// Sets default values
AAG_SpiderCharacter::AAG_SpiderCharacter()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AAG_SpiderCharacter::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AAG_SpiderCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

// Called to bind functionality to input
void AAG_SpiderCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

}

