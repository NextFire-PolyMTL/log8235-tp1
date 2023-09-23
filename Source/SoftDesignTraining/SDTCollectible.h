// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"
#include "Engine/StaticMeshActor.h"
#include "SDTCollectible.generated.h"



UCLASS()
class SOFTDESIGNTRAINING_API ASDTCollectible : public AStaticMeshActor
{
	GENERATED_BODY()
public:
    ASDTCollectible();

    void Collect();
    void OnCooldownDone();
    bool IsOnCooldown();

    // The duration taken by the collectible to respawn after being collected by an actor.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI, meta=(ClampMin = "0.0"))
    float CollectCooldownDuration = 10.f;

    // If checked, the collectible will move along the y axis (from left to right) using the specified Acceleration and MaxSpeed.
    // The semi-implicit Euler method is used to calculate the next speed and next position.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI)
    bool IsMoveable = false;

    // The maximum speed at which the collectible moves. The actual speed of the collectible
    // is calculated with the acceleration value.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI, meta = (ClampMin = "0.0"))
    float MaxSpeed = 500.0f;

    // The constant acceleration the collectible uses to calculate its next speed.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI, meta = (ClampMin = "1.0"))
    float Acceleration = 250.0f;

    // The distance from the wall at which the collectible must stop before moving
    // in the other direction.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = AI, meta = (ClampMin = "0.0"))
    float DistanceFromWall = 50.0f;

    virtual void Tick(float deltaTime) override;
    virtual void BeginPlay() override;

private:
    void Move(float deltaTime, FVector& spherePosAtZero, FVector& spherePotentialHitPoint);
    void ResetMovementComponents();

    FTimerHandle CollectCooldownTimer;
    FVector InitialPosition = FVector::ZeroVector;
    FVector CurrentSpeed = FVector::ZeroVector;
    FVector CurrentAcceleration = FVector::ZeroVector;
    FVector CurrentHitNormal = FVector::ZeroVector;
    FVector CurrentHitPoint = FVector::ZeroVector;
    float SphereRadius = 0.0f;
};
