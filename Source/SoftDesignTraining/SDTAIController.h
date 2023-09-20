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
    bool DetectCollectible(FVector& targetDirection);
    bool DetectWalls(FVector& targetDirection, float& collisionDistance);
    void ResetWallsDetection();
    void SpeedControl(float deltaTime, float wallCollisionDistance);
    void Move(float deltaTime, FVector targetDirection);
    bool IsInVisionCone(UWorld* world, AActor* pawn, AActor* targetActor);


    FVector lastImpactNormal = FVector::ZeroVector;
    FVector lastTargetDirectionForWalls = FVector::ZeroVector;
    int rotationDirection = 1;

    FVector previousRotationAxis = FVector::ZeroVector;
    FVector targetMoveTo;

    float TargetSpeed = 0.0f;
};
