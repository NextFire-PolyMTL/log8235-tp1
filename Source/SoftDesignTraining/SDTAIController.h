// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Components/SplineComponent.h"
#include "AIController.h"
#include "CoreMinimal.h"

#include "SDTAIController.generated.h"

/**
 *
 */
UCLASS(ClassGroup = AI, config = Game)
class SOFTDESIGNTRAINING_API ASDTAIController : public AAIController
{
    GENERATED_BODY()
public:
    virtual void BeginPlay() override;

    virtual void OnMoveCompleted(FAIRequestID RequestID, EPathFollowingResult::Type Result) override;

    virtual void Tick(float deltaTime) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)

    float MaxSpeed = 400.0f;

    float VisionDistance = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float VisionAngle = PI / 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float Acceleration = 250.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float ForwardWallRayCastDist = 190.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float SidesWallRayCastDist = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float RotationAngleBySecond = 200;

private:
    void CalculateFarForwardTarget(FVector headingTarget);
    void CalculateFarForwardTarget();
    void DetectCollectible();
    void DetectWalls();
    void SpeedControl(float deltaTime);
    void Move(float deltaTime);
    bool IsInVisionCone(UWorld* world, AActor* pawn, AActor* targetActor);

    bool isForwardHit = false;
    bool isTurningAround = false;
    float forwardImpactDistance = 0.0f;
    FVector lastImpactNormal = FVector::ZeroVector;
    int rotationDirection = 0;
    float TargetSpeed = 0.0f;

    FVector targetMoveTo;
};
