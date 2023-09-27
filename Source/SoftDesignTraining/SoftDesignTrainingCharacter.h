// Copyright 1998-2015 Epic Games, Inc. All Rights Reserved.
#pragma once
#include "GameFramework/Character.h"
#include "SoftDesignTrainingCharacter.generated.h"

UCLASS()
class ASoftDesignTrainingCharacter : public ACharacter
{
    GENERATED_BODY()

public:
    ASoftDesignTrainingCharacter();

    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float deltaTime) override;

    virtual void OnCollectPowerUp();
    virtual void OnDeath();

protected:
    UFUNCTION()
    virtual void OnBeginOverlap(UPrimitiveComponent *OverlappedComponent, AActor *OtherActor, UPrimitiveComponent *OtherComponent, int32 OtherBodyIndex, bool bFromSweep, const FHitResult &SweepResult);

    FVector StartingPosition;

    int PickupCount = 0;
    int DeathCount = 0;
};
